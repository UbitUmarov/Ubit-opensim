/*
 * Copyright (c) Contributors, http://opensimulator.org/
 * See CONTRIBUTORS.TXT for a full list of copyright holders.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the OpenSimulator Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE DEVELOPERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

using System;
using System.Collections.Generic;
using System.Net;
using System.Reflection;

using OpenSim.Framework;
using OpenSim.Services.Connectors.Friends;
using OpenSim.Services.Connectors.Hypergrid;
using OpenSim.Services.Interfaces;
using GridRegion = OpenSim.Services.Interfaces.GridRegion;
using OpenSim.Server.Base;
using FriendInfo = OpenSim.Services.Interfaces.FriendInfo;

using OpenMetaverse;
using log4net;
using Nini.Config;

namespace OpenSim.Services.HypergridService
{
    /// <summary>
    /// This service is for HG1.5 only, to make up for the fact that clients don't
    /// keep any private information in themselves, and that their 'home service'
    /// needs to do it for them.
    /// Once we have better clients, this shouldn't be needed.
    /// </summary>
    public class UserAgentService : IUserAgentService
    {
        private static readonly ILog m_log =
                LogManager.GetLogger(
                MethodBase.GetCurrentMethod().DeclaringType);

        // This will need to go into a DB table
        static Dictionary<UUID, TravelingAgentInfo> m_TravelingAgents = new Dictionary<UUID, TravelingAgentInfo>();

        static bool m_Initialized = false;

        protected static IGridUserService m_GridUserService;
        protected static IGridService m_GridService;
        protected static GatekeeperServiceConnector m_GatekeeperConnector;
        protected static IGatekeeperService m_GatekeeperService;
        protected static IFriendsService m_FriendsService;
        protected static IPresenceService m_PresenceService;
        protected static IUserAccountService m_UserAccountService;
        protected static IFriendsSimConnector m_FriendsLocalSimConnector; // standalone, points to HGFriendsModule
        protected static FriendsSimConnector m_FriendsSimConnector; // grid

        protected static string m_GridName;

        protected static bool m_BypassClientVerification;

        public UserAgentService(IConfigSource config) : this(config, null)
        {
        }

        public UserAgentService(IConfigSource config, IFriendsSimConnector friendsConnector)
        {
            // Let's set this always, because we don't know the sequence
            // of instantiations
            if (friendsConnector != null)
                m_FriendsLocalSimConnector = friendsConnector;

            if (!m_Initialized)
            {
                m_Initialized = true;

                m_log.DebugFormat("[HOME USERS SECURITY]: Starting...");

                m_FriendsSimConnector = new FriendsSimConnector();

                IConfig serverConfig = config.Configs["UserAgentService"];
                if (serverConfig == null)
                    throw new Exception(String.Format("No section UserAgentService in config file"));

                string gridService = serverConfig.GetString("GridService", String.Empty);
                string gridUserService = serverConfig.GetString("GridUserService", String.Empty);
                string gatekeeperService = serverConfig.GetString("GatekeeperService", String.Empty);
                string friendsService = serverConfig.GetString("FriendsService", String.Empty);
                string presenceService = serverConfig.GetString("PresenceService", String.Empty);
                string userAccountService = serverConfig.GetString("UserAccountService", String.Empty);

                m_BypassClientVerification = serverConfig.GetBoolean("BypassClientVerification", false);

                if (gridService == string.Empty || gridUserService == string.Empty || gatekeeperService == string.Empty)
                    throw new Exception(String.Format("Incomplete specifications, UserAgent Service cannot function."));

                Object[] args = new Object[] { config };
                m_GridService = ServerUtils.LoadPlugin<IGridService>(gridService, args);
                m_GridUserService = ServerUtils.LoadPlugin<IGridUserService>(gridUserService, args);
                m_GatekeeperConnector = new GatekeeperServiceConnector();
                m_GatekeeperService = ServerUtils.LoadPlugin<IGatekeeperService>(gatekeeperService, args);
                m_FriendsService = ServerUtils.LoadPlugin<IFriendsService>(friendsService, args);
                m_PresenceService = ServerUtils.LoadPlugin<IPresenceService>(presenceService, args);
                m_UserAccountService = ServerUtils.LoadPlugin<IUserAccountService>(userAccountService, args);

                m_GridName = serverConfig.GetString("ExternalName", string.Empty);
                if (m_GridName == string.Empty)
                {
                    serverConfig = config.Configs["GatekeeperService"];
                    m_GridName = serverConfig.GetString("ExternalName", string.Empty);
                }
                if (!m_GridName.EndsWith("/"))
                    m_GridName = m_GridName + "/";
            }
        }

        public GridRegion GetHomeRegion(UUID userID, out Vector3 position, out Vector3 lookAt)
        {
            position = new Vector3(128, 128, 0); lookAt = Vector3.UnitY;

            m_log.DebugFormat("[USER AGENT SERVICE]: Request to get home region of user {0}", userID);

            GridRegion home = null;
            GridUserInfo uinfo = m_GridUserService.GetGridUserInfo(userID.ToString());
            if (uinfo != null)
            {
                if (uinfo.HomeRegionID != UUID.Zero)
                {
                    home = m_GridService.GetRegionByUUID(UUID.Zero, uinfo.HomeRegionID);
                    position = uinfo.HomePosition;
                    lookAt = uinfo.HomeLookAt;
                }
                if (home == null)
                {
                    List<GridRegion> defs = m_GridService.GetDefaultRegions(UUID.Zero);
                    if (defs != null && defs.Count > 0)
                        home = defs[0];
                }
            }

            return home;
        }

        public bool LoginAgentToGrid(AgentCircuitData agentCircuit, GridRegion gatekeeper, GridRegion finalDestination, IPEndPoint clientIP, out string reason)
        {
            m_log.DebugFormat("[USER AGENT SERVICE]: Request to login user {0} {1} (@{2}) to grid {3}", 
                agentCircuit.firstname, agentCircuit.lastname, ((clientIP == null) ? "stored IP" : clientIP.Address.ToString()), gatekeeper.ServerURI);
            // Take the IP address + port of the gatekeeper (reg) plus the info of finalDestination
            GridRegion region = new GridRegion(gatekeeper);
            region.ServerURI = gatekeeper.ServerURI;
            region.ExternalHostName = finalDestination.ExternalHostName;
            region.InternalEndPoint = finalDestination.InternalEndPoint;
            region.RegionName = finalDestination.RegionName;
            region.RegionID = finalDestination.RegionID;
            region.RegionLocX = finalDestination.RegionLocX;
            region.RegionLocY = finalDestination.RegionLocY;

            // Generate a new service session
            agentCircuit.ServiceSessionID = region.ServerURI + ";" + UUID.Random();
            TravelingAgentInfo old = UpdateTravelInfo(agentCircuit, region);
            
            bool success = false;
            string myExternalIP = string.Empty;
            string gridName = gatekeeper.ServerURI;

            m_log.DebugFormat("[USER AGENT SERVICE]: this grid: {0}, desired grid: {1}", m_GridName, gridName);

            if (m_GridName == gridName)
                success = m_GatekeeperService.LoginAgent(agentCircuit, finalDestination, out reason);
            else
            {
                success = m_GatekeeperConnector.CreateAgent(region, agentCircuit, (uint)Constants.TeleportFlags.ViaLogin, out myExternalIP, out reason);
                if (success)
                    // Report them as nowhere
                    m_PresenceService.ReportAgent(agentCircuit.SessionID, UUID.Zero);
            }

            if (!success)
            {
                m_log.DebugFormat("[USER AGENT SERVICE]: Unable to login user {0} {1} to grid {2}, reason: {3}", 
                    agentCircuit.firstname, agentCircuit.lastname, region.ServerURI, reason);

                // restore the old travel info
                lock (m_TravelingAgents)
                {
                    if (old == null)
                        m_TravelingAgents.Remove(agentCircuit.SessionID);
                    else
                        m_TravelingAgents[agentCircuit.SessionID] = old;
                }

                return false;
            }

            m_log.DebugFormat("[USER AGENT SERVICE]: Gatekeeper sees me as {0}", myExternalIP);
            // else set the IP addresses associated with this client
            if (clientIP != null)
                m_TravelingAgents[agentCircuit.SessionID].ClientIPAddress = clientIP.Address.ToString();
            m_TravelingAgents[agentCircuit.SessionID].MyIpAddress = myExternalIP;

            return true;
        }

        public bool LoginAgentToGrid(AgentCircuitData agentCircuit, GridRegion gatekeeper, GridRegion finalDestination, out string reason)
        {
            reason = string.Empty;
            return LoginAgentToGrid(agentCircuit, gatekeeper, finalDestination, null, out reason);
        }

        private void SetClientIP(UUID sessionID, string ip)
        {
            if (m_TravelingAgents.ContainsKey(sessionID))
            {
                m_log.DebugFormat("[USER AGENT SERVICE]: Setting IP {0} for session {1}", ip, sessionID);
                m_TravelingAgents[sessionID].ClientIPAddress = ip;
            }
        }

        TravelingAgentInfo UpdateTravelInfo(AgentCircuitData agentCircuit, GridRegion region)
        {
            TravelingAgentInfo travel = new TravelingAgentInfo();
            TravelingAgentInfo old = null;
            lock (m_TravelingAgents)
            {
                if (m_TravelingAgents.ContainsKey(agentCircuit.SessionID))
                {
                    // Very important! Override whatever this agent comes with.
                    // UserAgentService always sets the IP for every new agent
                    // with the original IP address.
                    agentCircuit.IPAddress = m_TravelingAgents[agentCircuit.SessionID].ClientIPAddress;

                    old = m_TravelingAgents[agentCircuit.SessionID];
                }

                m_TravelingAgents[agentCircuit.SessionID] = travel;
            }
            travel.UserID = agentCircuit.AgentID;
            travel.GridExternalName = region.ServerURI;
            travel.ServiceToken = agentCircuit.ServiceSessionID;
            if (old != null)
                travel.ClientIPAddress = old.ClientIPAddress;

            return old;
        }

        public void LogoutAgent(UUID userID, UUID sessionID)
        {
            m_log.DebugFormat("[USER AGENT SERVICE]: User {0} logged out", userID);

            lock (m_TravelingAgents)
            {
                List<UUID> travels = new List<UUID>();
                foreach (KeyValuePair<UUID, TravelingAgentInfo> kvp in m_TravelingAgents)
                    if (kvp.Value == null) // do some clean up
                        travels.Add(kvp.Key);
                    else if (kvp.Value.UserID == userID)
                        travels.Add(kvp.Key);
                foreach (UUID session in travels)
                    m_TravelingAgents.Remove(session);
            }

            GridUserInfo guinfo = m_GridUserService.GetGridUserInfo(userID.ToString());
            if (guinfo != null)
                m_GridUserService.LoggedOut(userID.ToString(), sessionID, guinfo.LastRegionID, guinfo.LastPosition, guinfo.LastLookAt);
        }

        // We need to prevent foreign users with the same UUID as a local user
        public bool IsAgentComingHome(UUID sessionID, string thisGridExternalName)
        {
            if (!m_TravelingAgents.ContainsKey(sessionID))
                return false;

            TravelingAgentInfo travel = m_TravelingAgents[sessionID];

            return travel.GridExternalName.ToLower() == thisGridExternalName.ToLower();
        }

        public bool VerifyClient(UUID sessionID, string reportedIP)
        {
            if (m_BypassClientVerification)
                return true;

            m_log.DebugFormat("[USER AGENT SERVICE]: Verifying Client session {0} with reported IP {1}.", 
                sessionID, reportedIP);

            if (m_TravelingAgents.ContainsKey(sessionID))
            {
                bool result = m_TravelingAgents[sessionID].ClientIPAddress == reportedIP ||
                    m_TravelingAgents[sessionID].MyIpAddress == reportedIP; // NATed

                m_log.DebugFormat("[USER AGENT SERVICE]: Comparing {0} with login IP {1} and MyIP {1}; result is {3}",
                                    reportedIP, m_TravelingAgents[sessionID].ClientIPAddress, m_TravelingAgents[sessionID].MyIpAddress, result);

                return result;
            }

            return false;
        }

        public bool VerifyAgent(UUID sessionID, string token)
        {
            if (m_TravelingAgents.ContainsKey(sessionID))
            {
                m_log.DebugFormat("[USER AGENT SERVICE]: Verifying agent token {0} against {1}", token, m_TravelingAgents[sessionID].ServiceToken);
                return m_TravelingAgents[sessionID].ServiceToken == token;
            }

            m_log.DebugFormat("[USER AGENT SERVICE]: Token verification for session {0}: no such session", sessionID);

            return false;
        }

        public List<UUID> StatusNotification(List<string> friends, UUID foreignUserID, bool online)
        {
            if (m_FriendsService == null || m_PresenceService == null)
            {
                m_log.WarnFormat("[USER AGENT SERVICE]: Unable to perform status notifications because friends or presence services are missing");
                return new List<UUID>();
            }

            List<UUID> localFriendsOnline = new List<UUID>();

            m_log.DebugFormat("[USER AGENT SERVICE]: Status notification: foreign user {0} wants to notify {1} local friends", foreignUserID, friends.Count);

            // First, let's double check that the reported friends are, indeed, friends of that user
            // And let's check that the secret matches
            List<string> usersToBeNotified = new List<string>();
            foreach (string uui in friends)
            {
                UUID localUserID;
                string secret = string.Empty, tmp = string.Empty;
                if (Util.ParseUniversalUserIdentifier(uui, out localUserID, out tmp, out tmp, out tmp, out secret))
                {
                    FriendInfo[] friendInfos = m_FriendsService.GetFriends(localUserID);
                    foreach (FriendInfo finfo in friendInfos)
                    {
                        if (finfo.Friend.StartsWith(foreignUserID.ToString()) && finfo.Friend.EndsWith(secret))
                        {
                            // great!
                            usersToBeNotified.Add(localUserID.ToString());
                        }
                    }
                }
            }

            // Now, let's send the notifications
            m_log.DebugFormat("[USER AGENT SERVICE]: Status notification: user has {0} local friends", usersToBeNotified.Count);

            // First, let's send notifications to local users who are online in the home grid
            PresenceInfo[] friendSessions = m_PresenceService.GetAgents(usersToBeNotified.ToArray());
            if (friendSessions != null && friendSessions.Length > 0)
            {
                PresenceInfo friendSession = null;
                foreach (PresenceInfo pinfo in friendSessions)
                    if (pinfo.RegionID != UUID.Zero) // let's guard against traveling agents
                    {
                        friendSession = pinfo;
                        break;
                    }

                if (friendSession != null)
                {
                    ForwardStatusNotificationToSim(friendSession.RegionID, foreignUserID, friendSession.UserID, online);
                    usersToBeNotified.Remove(friendSession.UserID.ToString());
                    UUID id;
                    if (UUID.TryParse(friendSession.UserID, out id))
                        localFriendsOnline.Add(id);

                }
            }

            // Lastly, let's notify the rest who may be online somewhere else
            foreach (string user in usersToBeNotified)
            {
                UUID id = new UUID(user);
                if (m_TravelingAgents.ContainsKey(id) && m_TravelingAgents[id].GridExternalName != m_GridName)
                {
                    string url = m_TravelingAgents[id].GridExternalName;
                    // forward
                    m_log.WarnFormat("[USER AGENT SERVICE]: User {0} is visiting {1}. HG Status notifications still not implemented.", user, url);
                }
            }

            // and finally, let's send the online friends
            if (online)
            {
                return localFriendsOnline;
            }
            else
                return new List<UUID>();
        }

        protected void ForwardStatusNotificationToSim(UUID regionID, UUID foreignUserID, string user, bool online)
        {
            UUID userID;
            if (UUID.TryParse(user, out userID))
            {
                if (m_FriendsLocalSimConnector != null)
                {
                    m_log.DebugFormat("[USER AGENT SERVICE]: Local Notify, user {0} is {1}", foreignUserID, (online ? "online" : "offline"));
                    m_FriendsLocalSimConnector.StatusNotify(foreignUserID, userID, online);
                }
                else
                {
                    GridRegion region = m_GridService.GetRegionByUUID(UUID.Zero /* !!! */, regionID);
                    if (region != null)
                    {
                        m_log.DebugFormat("[USER AGENT SERVICE]: Remote Notify to region {0}, user {1} is {2}", region.RegionName, foreignUserID, (online ? "online" : "offline"));
                        m_FriendsSimConnector.StatusNotify(region, foreignUserID, userID, online);
                    }
                }
            }
        }

        public List<UUID> GetOnlineFriends(UUID foreignUserID, List<string> friends)
        {
            List<UUID> online = new List<UUID>();

            if (m_FriendsService == null || m_PresenceService == null)
            {
                m_log.WarnFormat("[USER AGENT SERVICE]: Unable to get online friends because friends or presence services are missing");
                return online;
            }

            m_log.DebugFormat("[USER AGENT SERVICE]: Foreign user {0} wants to know status of {1} local friends", foreignUserID, friends.Count);

            // First, let's double check that the reported friends are, indeed, friends of that user
            // And let's check that the secret matches and the rights
            List<string> usersToBeNotified = new List<string>();
            foreach (string uui in friends)
            {
                UUID localUserID;
                string secret = string.Empty, tmp = string.Empty;
                if (Util.ParseUniversalUserIdentifier(uui, out localUserID, out tmp, out tmp, out tmp, out secret))
                {
                    FriendInfo[] friendInfos = m_FriendsService.GetFriends(localUserID);
                    foreach (FriendInfo finfo in friendInfos)
                    {
                        if (finfo.Friend.StartsWith(foreignUserID.ToString()) && finfo.Friend.EndsWith(secret) && 
                            (finfo.TheirFlags & (int)FriendRights.CanSeeOnline) != 0 && (finfo.TheirFlags != -1))
                        {
                            // great!
                            usersToBeNotified.Add(localUserID.ToString());
                        }
                    }
                }
            }

            // Now, let's find out their status
            m_log.DebugFormat("[USER AGENT SERVICE]: GetOnlineFriends: user has {0} local friends with status rights", usersToBeNotified.Count);

            // First, let's send notifications to local users who are online in the home grid
            PresenceInfo[] friendSessions = m_PresenceService.GetAgents(usersToBeNotified.ToArray());
            if (friendSessions != null && friendSessions.Length > 0)
            {
                foreach (PresenceInfo pi in friendSessions)
                {
                    UUID presenceID;
                    if (UUID.TryParse(pi.UserID, out presenceID))
                        online.Add(presenceID);
                }
            }

            return online;
        }

        public Dictionary<string, object> GetServerURLs(UUID userID)
        {
            if (m_UserAccountService == null)
            {
                m_log.WarnFormat("[USER AGENT SERVICE]: Unable to get server URLs because user account service is missing");
                return new Dictionary<string, object>();
            }
            UserAccount account = m_UserAccountService.GetUserAccount(UUID.Zero /*!!!*/, userID);
            if (account != null)
                return account.ServiceURLs;

            return new Dictionary<string, object>();
        }

        public string LocateUser(UUID userID)
        {
            foreach (TravelingAgentInfo t in m_TravelingAgents.Values)
            {
                if (t == null)
                {
                    m_log.ErrorFormat("[USER AGENT SERVICE]: Oops! Null TravelingAgentInfo. Please report this on mantis");
                    continue;
                }
                if (t.UserID == userID && !m_GridName.Equals(t.GridExternalName))
                    return t.GridExternalName;
            }

            return string.Empty;
        }

        public string GetUUI(UUID userID, UUID targetUserID)
        {
            // Let's see if it's a local user
            UserAccount account = m_UserAccountService.GetUserAccount(UUID.Zero, targetUserID);
            if (account != null)
                return targetUserID.ToString() + ";" + m_GridName + ";" + account.FirstName + " " + account.LastName ;

            // Let's try the list of friends
            FriendInfo[] friends = m_FriendsService.GetFriends(userID);
            if (friends != null && friends.Length > 0)
            {
                foreach (FriendInfo f in friends)
                    if (f.Friend.StartsWith(targetUserID.ToString()))
                    {
                        // Let's remove the secret 
                        UUID id; string tmp = string.Empty, secret = string.Empty;
                        if (Util.ParseUniversalUserIdentifier(f.Friend, out id, out tmp, out tmp, out tmp, out secret))
                            return f.Friend.Replace(secret, "0");
                    }
            }

            return string.Empty;
        }
    }

    class TravelingAgentInfo
    {
        public UUID UserID;
        public string GridExternalName = string.Empty;
        public string ServiceToken = string.Empty;
        public string ClientIPAddress = string.Empty; // as seen from this user agent service
        public string MyIpAddress = string.Empty; // the user agent service's external IP, as seen from the next gatekeeper
    }

}
