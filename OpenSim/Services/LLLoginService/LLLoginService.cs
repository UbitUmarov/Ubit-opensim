﻿using System;
using System.Collections.Generic;
using System.Net;
using System.Reflection;
using System.Text.RegularExpressions;

using log4net;
using Nini.Config;
using OpenMetaverse;

using OpenSim.Framework;
using OpenSim.Framework.Capabilities;
using OpenSim.Server.Base;
using OpenSim.Services.Interfaces;
using GridRegion = OpenSim.Services.Interfaces.GridRegion;

namespace OpenSim.Services.LLLoginService
{
    public class LLLoginService : ILoginService
    {
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private IUserAccountService m_UserAccountService;
        private IAuthenticationService m_AuthenticationService;
        private IInventoryService m_InventoryService;
        private IGridService m_GridService;
        private IPresenceService m_PresenceService;
        private ISimulationService m_LocalSimulationService;

        private string m_DefaultRegionName;
        private string m_RemoteSimulationDll;
        private string m_WelcomeMessage;
        private bool m_RequireInventory;

        public LLLoginService(IConfigSource config, ISimulationService simService)
        {
            IConfig serverConfig = config.Configs["LoginService"];
            if (serverConfig == null)
                throw new Exception(String.Format("No section LoginService in config file"));

            string accountService = serverConfig.GetString("UserAccountService", String.Empty);
            string authService = serverConfig.GetString("AuthenticationService", String.Empty);
            string invService = serverConfig.GetString("InventoryService", String.Empty);
            string gridService = serverConfig.GetString("GridService", String.Empty);
            string presenceService = serverConfig.GetString("PresenceService", String.Empty);

            m_DefaultRegionName = serverConfig.GetString("DefaultRegion", String.Empty);
            m_RemoteSimulationDll = serverConfig.GetString("RemoteSimulationService", String.Empty);
            m_WelcomeMessage = serverConfig.GetString("WelcomeMessage", "Welcome to OpenSim!");
            m_RequireInventory = serverConfig.GetBoolean("RequireInventory", true);

            // These 3 are required; the other 2 aren't
            if (accountService == string.Empty || authService == string.Empty ||
                invService == string.Empty)
                throw new Exception("LoginService is missing service specifications");

            Object[] args = new Object[] { config };
            m_UserAccountService = ServerUtils.LoadPlugin<IUserAccountService>(accountService, args);
            m_AuthenticationService = ServerUtils.LoadPlugin<IAuthenticationService>(authService, args);
            m_InventoryService = ServerUtils.LoadPlugin<IInventoryService>(invService, args);
            if (gridService != string.Empty)
                m_GridService = ServerUtils.LoadPlugin<IGridService>(gridService, args);
            if (presenceService != string.Empty)
                m_PresenceService = ServerUtils.LoadPlugin<IPresenceService>(presenceService, args);
            m_LocalSimulationService = simService;

        }

        public LLLoginService(IConfigSource config) : this(config, null)
        {
        }

        public LoginResponse Login(string firstName, string lastName, string passwd, string startLocation, IPEndPoint clientIP)
        {
            bool success = false;

            // Get the account and check that it exists
            UserAccount account = m_UserAccountService.GetUserAccount(UUID.Zero, firstName, lastName);
            if (account == null)
            {
                m_log.InfoFormat("[LLOGIN SERVICE]: Login failed, reason: user not found");
                return LLFailedLoginResponse.UserProblem;
            }

            // Authenticate this user
            string token = m_AuthenticationService.Authenticate(account.PrincipalID, passwd, 30);
            UUID secureSession = UUID.Zero;
            if ((token == string.Empty) || (token != string.Empty && !UUID.TryParse(token, out secureSession)))
            {
                m_log.InfoFormat("[LLOGIN SERVICE]: Login failed, reason: authentication failed");
                return LLFailedLoginResponse.UserProblem;
            }

            // Get the user's inventory
            List<InventoryFolderBase> inventorySkel = m_InventoryService.GetInventorySkeleton(account.PrincipalID);
            if (m_RequireInventory && ((inventorySkel == null) || (inventorySkel != null && inventorySkel.Count == 0)))
            {
                m_log.InfoFormat("[LLOGIN SERVICE]: Login failed, reason: unable to retrieve user inventory");
                return LLFailedLoginResponse.InventoryProblem;
            }

            // Login the presence
            // We may want to check for user already logged in, to
            // stay compatible with what people expect...
            UUID session = UUID.Random();
            PresenceInfo presence = null;
            GridRegion home = null;
            if (m_PresenceService != null)
            {
                success = m_PresenceService.LoginAgent(account.PrincipalID.ToString(), session, secureSession);
                if (!success)
                {
                    m_log.InfoFormat("[LLOGIN SERVICE]: Login failed, reason: could not login presence");
                    return LLFailedLoginResponse.GridProblem;
                }
                // Get the updated presence info
                presence = m_PresenceService.GetAgent(session);

                // Get the home region
                if ((presence.HomeRegionID != UUID.Zero) && m_GridService != null)
                {
                    home = m_GridService.GetRegionByUUID(account.ScopeID, presence.HomeRegionID);
                }
            }

            // Find the destination region/grid
            string where = string.Empty;
            Vector3 position = Vector3.Zero;
            Vector3 lookAt = Vector3.Zero;
            GridRegion destination = FindDestination(account, presence, session, startLocation, out where, out position, out lookAt);
            if (destination == null)
            {
                m_PresenceService.LogoutAgent(session);
                m_log.InfoFormat("[LLOGIN SERVICE]: Login failed, reason: destination not found");
                return LLFailedLoginResponse.GridProblem;
            }

            // Instantiate/get the simulation interface and launch an agent at the destination
            ISimulationService simConnector = null;
            string reason = string.Empty;
            uint circuitCode = 0;
            AgentCircuitData aCircuit = null;
            Object[] args = new Object[] { destination };
            // HG standalones have both a localSimulatonDll and a remoteSimulationDll
            // non-HG standalones have just a localSimulationDll
            // independent login servers have just a remoteSimulationDll
            if (!startLocation.Contains("@") && (m_LocalSimulationService != null))
                simConnector = m_LocalSimulationService;
            else if (m_RemoteSimulationDll != string.Empty)
                simConnector = ServerUtils.LoadPlugin<ISimulationService>(m_RemoteSimulationDll, args);
            if (simConnector != null)
            {
                circuitCode = (uint)Util.RandomClass.Next(); ;
                aCircuit = LaunchAgent(simConnector, destination, account, session, secureSession, circuitCode, position, out reason);
            }
            if (aCircuit == null)
            {
                m_PresenceService.LogoutAgent(session);
                m_log.InfoFormat("[LLOGIN SERVICE]: Login failed, reason: {0}", reason);
                return LLFailedLoginResponse.GridProblem;
            }

            // TODO: Get Friends list... 

            // Finally, fill out the response and return it
            LLLoginResponse response = new LLLoginResponse(account, aCircuit, presence, destination, inventorySkel, 
                where, startLocation, position, lookAt, m_WelcomeMessage, home, clientIP);

            return response;
        }

        private GridRegion FindDestination(UserAccount account, PresenceInfo pinfo, UUID sessionID, string startLocation, out string where, out Vector3 position, out Vector3 lookAt)
        {
            where = "home";
            position = new Vector3(128, 128, 0);
            lookAt = new Vector3(0, 1, 0);
            if (startLocation.Equals("home"))
            {
                // logging into home region
                if (m_PresenceService == null || m_GridService == null)
                    return null;

                if (pinfo == null)
                    return null;

                GridRegion region = null;

                if (pinfo.HomeRegionID.Equals(UUID.Zero))
                    region = m_GridService.GetRegionByName(account.ScopeID, m_DefaultRegionName);
                else
                    region = m_GridService.GetRegionByUUID(account.ScopeID, pinfo.HomeRegionID);

                return region;
            }
            else if (startLocation.Equals("last"))
            {
                // logging into last visited region
                where = "last";
                if (m_PresenceService == null || m_GridService == null)
                    return null;

                if (pinfo == null)
                    return null;

                GridRegion region = null;

                if (pinfo.RegionID.Equals(UUID.Zero))
                    region = m_GridService.GetRegionByName(account.ScopeID, m_DefaultRegionName);
                else
                {
                    region = m_GridService.GetRegionByUUID(account.ScopeID, pinfo.RegionID);
                    position = pinfo.Position;
                    lookAt = pinfo.LookAt;
                }
                return region;

            }
            else
            {
                // free uri form
                // e.g. New Moon&135&46  New Moon@osgrid.org:8002&153&34
                where = "url";
                Regex reURI = new Regex(@"^uri:(?<region>[^&]+)&(?<x>\d+)&(?<y>\d+)&(?<z>\d+)$");
                Match uriMatch = reURI.Match(startLocation);
                if (uriMatch == null)
                {
                    m_log.InfoFormat("[LLLOGIN SERVICE]: Got Custom Login URI {0}, but can't process it", startLocation);
                    return null;
                }
                else
                {
                    position = new Vector3(float.Parse(uriMatch.Groups["x"].Value),
                                           float.Parse(uriMatch.Groups["y"].Value),
                                           float.Parse(uriMatch.Groups["z"].Value));

                    string regionName = uriMatch.Groups["region"].ToString();
                    if (regionName != null)
                    {
                        if (!regionName.Contains("@"))
                        {
                            List<GridRegion> regions = m_GridService.GetRegionsByName(account.ScopeID, regionName, 1);
                            if ((regions == null) || (regions != null && regions.Count == 0))
                            {
                                m_log.InfoFormat("[LLLOGIN SERVICE]: Got Custom Login URI {0}, can't locate region {1}", startLocation, regionName);
                                return null;
                            }
                            return regions[0];
                        }
                        else
                        {
                            string[] parts = regionName.Split(new char[] { '@' });
                            if (parts.Length < 2)
                            {
                                m_log.InfoFormat("[LLLOGIN SERVICE]: Got Custom Login URI {0}, can't locate region {1}", startLocation, regionName);
                                return null;
                            }
                            // Valid specification of a remote grid
                            regionName = parts[0];
                            string domainLocator = parts[1];
                            parts = domainLocator.Split(new char[] {':'});
                            string domainName = parts[0];
                            uint port = 0;
                            if (parts.Length > 1)
                                UInt32.TryParse(parts[1], out port);
                            GridRegion region = new GridRegion();
                            region.ExternalHostName = domainName;
                            region.HttpPort = port;
                            region.RegionName = regionName;
                            return region;
                        }

                    }
                    else
                    {
                        if (m_PresenceService == null || m_GridService == null)
                            return null;

                        return m_GridService.GetRegionByName(account.ScopeID, m_DefaultRegionName);

                    }
                }
                //response.LookAt = "[r0,r1,r0]";
                //// can be: last, home, safe, url
                //response.StartLocation = "url";

            }

        }

        private AgentCircuitData LaunchAgent(ISimulationService simConnector, GridRegion region, UserAccount account, 
            UUID session, UUID secureSession, uint circuit, Vector3 position, out string reason)
        {
            reason = string.Empty;
            AgentCircuitData aCircuit = new AgentCircuitData();

            aCircuit.AgentID = account.PrincipalID;
            //aCircuit.Appearance = optional
            //aCircuit.BaseFolder = irrelevant
            aCircuit.CapsPath = CapsUtil.GetRandomCapsObjectPath();
            aCircuit.child = false;
            aCircuit.circuitcode = circuit;
            aCircuit.firstname = account.FirstName;
            //aCircuit.InventoryFolder = irrelevant
            aCircuit.lastname = account.LastName;
            aCircuit.SecureSessionID = secureSession;
            aCircuit.SessionID = session;
            aCircuit.startpos = position;

            if (simConnector.CreateAgent(region.RegionHandle, aCircuit, 0, out reason))
                return aCircuit;

            return null;

        }
    }
}