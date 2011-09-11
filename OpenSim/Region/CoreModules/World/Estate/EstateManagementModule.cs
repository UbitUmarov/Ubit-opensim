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
using System.IO;
using System.Reflection;
using System.Security;
using log4net;
using Mono.Addins;
using Nini.Config;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;

namespace OpenSim.Region.CoreModules.World.Estate
{
    [Extension(Path = "/OpenSim/RegionModules", NodeName = "RegionModule", Id = "EstateManagementModule")]
    public class EstateManagementModule : IEstateModule, INonSharedRegionModule
    {
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private delegate void LookupUUIDS(List<UUID> uuidLst);

        public Scene Scene { get; private set; }
        public IUserManagement UserManager { get; private set; }
        
        protected EstateManagementCommands m_commands;                

        private EstateTerrainXferHandler TerrainUploader;

        public event ChangeDelegate OnRegionInfoChange;
        public event ChangeDelegate OnEstateInfoChange;
        public event MessageDelegate OnEstateMessage;

        #region Packet Data Responders

        private void sendDetailedEstateData(IClientAPI remote_client, UUID invoice)
        {
            uint sun = 0;

            if (!Scene.RegionInfo.EstateSettings.UseGlobalTime)
                sun = (uint)(Scene.RegionInfo.EstateSettings.SunPosition * 1024.0) + 0x1800;
            UUID estateOwner;
            estateOwner = Scene.RegionInfo.EstateSettings.EstateOwner;

            if (Scene.Permissions.IsGod(remote_client.AgentId))
                estateOwner = remote_client.AgentId;

            remote_client.SendDetailedEstateData(invoice,
                    Scene.RegionInfo.EstateSettings.EstateName,
                    Scene.RegionInfo.EstateSettings.EstateID,
                    Scene.RegionInfo.EstateSettings.ParentEstateID,
                    GetEstateFlags(),
                    sun,
                    Scene.RegionInfo.RegionSettings.Covenant,
                    Scene.RegionInfo.EstateSettings.AbuseEmail,
                    estateOwner);

            remote_client.SendEstateList(invoice,
                    (int)Constants.EstateAccessCodex.EstateManagers,
                    Scene.RegionInfo.EstateSettings.EstateManagers,
                    Scene.RegionInfo.EstateSettings.EstateID);

            remote_client.SendEstateList(invoice,
                    (int)Constants.EstateAccessCodex.AccessOptions,
                    Scene.RegionInfo.EstateSettings.EstateAccess,
                    Scene.RegionInfo.EstateSettings.EstateID);

            remote_client.SendEstateList(invoice,
                    (int)Constants.EstateAccessCodex.AllowedGroups,
                    Scene.RegionInfo.EstateSettings.EstateGroups,
                    Scene.RegionInfo.EstateSettings.EstateID);

            remote_client.SendBannedUserList(invoice,
                    Scene.RegionInfo.EstateSettings.EstateBans,
                    Scene.RegionInfo.EstateSettings.EstateID);
        }

        private void estateSetRegionInfoHandler(bool blockTerraform, bool noFly, bool allowDamage, bool blockLandResell, int maxAgents, float objectBonusFactor,
                                                int matureLevel, bool restrictPushObject, bool allowParcelChanges)
        {
            if (blockTerraform)
                Scene.RegionInfo.RegionSettings.BlockTerraform = true;
            else
                Scene.RegionInfo.RegionSettings.BlockTerraform = false;

            if (noFly)
                Scene.RegionInfo.RegionSettings.BlockFly = true;
            else
                Scene.RegionInfo.RegionSettings.BlockFly = false;

            if (allowDamage)
                Scene.RegionInfo.RegionSettings.AllowDamage = true;
            else
                Scene.RegionInfo.RegionSettings.AllowDamage = false;

            if (blockLandResell)
                Scene.RegionInfo.RegionSettings.AllowLandResell = false;
            else
                Scene.RegionInfo.RegionSettings.AllowLandResell = true;

            if((byte)maxAgents <= Scene.RegionInfo.AgentCapacity)
                Scene.RegionInfo.RegionSettings.AgentLimit = (byte) maxAgents;
			else
                Scene.RegionInfo.RegionSettings.AgentLimit = Scene.RegionInfo.AgentCapacity;

            Scene.RegionInfo.RegionSettings.ObjectBonus = objectBonusFactor;

            if (matureLevel <= 13)
                Scene.RegionInfo.RegionSettings.Maturity = 0;
            else if (matureLevel <= 21)
                Scene.RegionInfo.RegionSettings.Maturity = 1;
            else
                Scene.RegionInfo.RegionSettings.Maturity = 2;

            if (restrictPushObject)
                Scene.RegionInfo.RegionSettings.RestrictPushing = true;
            else
                Scene.RegionInfo.RegionSettings.RestrictPushing = false;

            if (allowParcelChanges)
                Scene.RegionInfo.RegionSettings.AllowLandJoinDivide = true;
            else
                Scene.RegionInfo.RegionSettings.AllowLandJoinDivide = false;

            Scene.RegionInfo.RegionSettings.Save();
            TriggerRegionInfoChange();

            sendRegionInfoPacketToAll();
        }

        public void setEstateTerrainBaseTexture(IClientAPI remoteClient, int corner, UUID texture)
        {
            if (texture == UUID.Zero)
                return;

            switch (corner)
            {
                case 0:
                    Scene.RegionInfo.RegionSettings.TerrainTexture1 = texture;
                    break;
                case 1:
                    Scene.RegionInfo.RegionSettings.TerrainTexture2 = texture;
                    break;
                case 2:
                    Scene.RegionInfo.RegionSettings.TerrainTexture3 = texture;
                    break;
                case 3:
                    Scene.RegionInfo.RegionSettings.TerrainTexture4 = texture;
                    break;
            }
            Scene.RegionInfo.RegionSettings.Save();
            TriggerRegionInfoChange();
            sendRegionInfoPacketToAll();
        }

        public void setEstateTerrainTextureHeights(IClientAPI client, int corner, float lowValue, float highValue)
        {
            switch (corner)
            {
                case 0:
                    Scene.RegionInfo.RegionSettings.Elevation1SW = lowValue;
                    Scene.RegionInfo.RegionSettings.Elevation2SW = highValue;
                    break;
                case 1:
                    Scene.RegionInfo.RegionSettings.Elevation1NW = lowValue;
                    Scene.RegionInfo.RegionSettings.Elevation2NW = highValue;
                    break;
                case 2:
                    Scene.RegionInfo.RegionSettings.Elevation1SE = lowValue;
                    Scene.RegionInfo.RegionSettings.Elevation2SE = highValue;
                    break;
                case 3:
                    Scene.RegionInfo.RegionSettings.Elevation1NE = lowValue;
                    Scene.RegionInfo.RegionSettings.Elevation2NE = highValue;
                    break;
            }
            Scene.RegionInfo.RegionSettings.Save();
            TriggerRegionInfoChange();
            sendRegionHandshakeToAll();
            sendRegionInfoPacketToAll();
        }

        private void handleCommitEstateTerrainTextureRequest(IClientAPI remoteClient)
        {
            // sendRegionHandshakeToAll();
        }

        public void setRegionTerrainSettings(float WaterHeight,
                float TerrainRaiseLimit, float TerrainLowerLimit,
                bool UseEstateSun, bool UseFixedSun, float SunHour,
                bool UseGlobal, bool EstateFixedSun, float EstateSunHour)
        {
            // Water Height
            Scene.RegionInfo.RegionSettings.WaterHeight = WaterHeight;

            // Terraforming limits
            Scene.RegionInfo.RegionSettings.TerrainRaiseLimit = TerrainRaiseLimit;
            Scene.RegionInfo.RegionSettings.TerrainLowerLimit = TerrainLowerLimit;

            // Time of day / fixed sun
            Scene.RegionInfo.RegionSettings.UseEstateSun = UseEstateSun;
            Scene.RegionInfo.RegionSettings.FixedSun = UseFixedSun;
            Scene.RegionInfo.RegionSettings.SunPosition = SunHour;

            Scene.TriggerEstateSunUpdate();

            //m_log.Debug("[ESTATE]: UFS: " + UseFixedSun.ToString());
            //m_log.Debug("[ESTATE]: SunHour: " + SunHour.ToString());

            sendRegionInfoPacketToAll();
            Scene.RegionInfo.RegionSettings.Save();
            TriggerRegionInfoChange();
        }

        private void handleEstateRestartSimRequest(IClientAPI remoteClient, int timeInSeconds)
        {
            IRestartModule restartModule = Scene.RequestModuleInterface<IRestartModule>();
            if (restartModule != null)
            {
                List<int> times = new List<int>();
                while (timeInSeconds > 0)
                {
                    times.Add(timeInSeconds);
                    if (timeInSeconds > 300)
                        timeInSeconds -= 120;
                    else if (timeInSeconds > 30)
                        timeInSeconds -= 30;
                    else
                        timeInSeconds -= 15;
                }

                restartModule.ScheduleRestart(UUID.Zero, "Region will restart in {0}", times.ToArray(), true);
            }
        }

        private void handleChangeEstateCovenantRequest(IClientAPI remoteClient, UUID estateCovenantID)
        {
//            m_log.DebugFormat(
//                "[ESTATE MANAGEMENT MODULE]: Handling request from {0} to change estate covenant to {1}", 
//                remoteClient.Name, estateCovenantID);
            
            Scene.RegionInfo.RegionSettings.Covenant = estateCovenantID;
            Scene.RegionInfo.RegionSettings.Save();
            TriggerRegionInfoChange();
        }

        private void handleEstateAccessDeltaRequest(IClientAPI remote_client, UUID invoice, int estateAccessType, UUID user)
        {
            // EstateAccessDelta handles Estate Managers, Sim Access, Sim Banlist, allowed Groups..  etc.

            if (user == Scene.RegionInfo.EstateSettings.EstateOwner)
                return; // never process EO

            if ((estateAccessType & 4) != 0) // User add
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, true) || Scene.Permissions.BypassPermissions())
                {
                    if ((estateAccessType & 1) != 0) // All estates
                    {
                        List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                        EstateSettings estateSettings;

                        foreach (int estateID in estateIDs)
                        {
                            if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                            {
                                estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                estateSettings.AddEstateUser(user);
                                estateSettings.Save();
                            }
                        }
                    }

                    Scene.RegionInfo.EstateSettings.AddEstateUser(user);
                    Scene.RegionInfo.EstateSettings.Save();

                    TriggerEstateInfoChange();
                    remote_client.SendEstateList(invoice, (int)Constants.EstateAccessCodex.AccessOptions, Scene.RegionInfo.EstateSettings.EstateAccess, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }

            }
            if ((estateAccessType & 8) != 0) // User remove
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, true) || Scene.Permissions.BypassPermissions())
                {
                    if ((estateAccessType & 1) != 0) // All estates
                    {
                        List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                        EstateSettings estateSettings;

                        foreach (int estateID in estateIDs)
                        {
                            if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                            {
                                estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                estateSettings.RemoveEstateUser(user);
                                estateSettings.Save();
                            }
                        }
                    }

                    Scene.RegionInfo.EstateSettings.RemoveEstateUser(user);
                    Scene.RegionInfo.EstateSettings.Save();

                    TriggerEstateInfoChange();
                    remote_client.SendEstateList(invoice, (int)Constants.EstateAccessCodex.AccessOptions, Scene.RegionInfo.EstateSettings.EstateAccess, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }
            }
            if ((estateAccessType & 16) != 0) // Group add
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, true) || Scene.Permissions.BypassPermissions())
                {
                    if ((estateAccessType & 1) != 0) // All estates
                    {
                        List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                        EstateSettings estateSettings;

                        foreach (int estateID in estateIDs)
                        {
                            if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                            {
                                estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                estateSettings.AddEstateGroup(user);
                                estateSettings.Save();
                            }
                        }
                    }

                    Scene.RegionInfo.EstateSettings.AddEstateGroup(user);
                    Scene.RegionInfo.EstateSettings.Save();

                    TriggerEstateInfoChange();
                    remote_client.SendEstateList(invoice, (int)Constants.EstateAccessCodex.AllowedGroups, Scene.RegionInfo.EstateSettings.EstateGroups, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }
            }
            if ((estateAccessType & 32) != 0) // Group remove
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, true) || Scene.Permissions.BypassPermissions())
                {
                    if ((estateAccessType & 1) != 0) // All estates
                    {
                        List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                        EstateSettings estateSettings;

                        foreach (int estateID in estateIDs)
                        {
                            if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                            {
                                estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                estateSettings.RemoveEstateGroup(user);
                                estateSettings.Save();
                            }
                        }
                    }

                    Scene.RegionInfo.EstateSettings.RemoveEstateGroup(user);
                    Scene.RegionInfo.EstateSettings.Save();

                    TriggerEstateInfoChange();
                    remote_client.SendEstateList(invoice, (int)Constants.EstateAccessCodex.AllowedGroups, Scene.RegionInfo.EstateSettings.EstateGroups, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }
            }
            if ((estateAccessType & 64) != 0) // Ban add
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, false) || Scene.Permissions.BypassPermissions())
                {
                    EstateBan[] banlistcheck = Scene.RegionInfo.EstateSettings.EstateBans;

                    bool alreadyInList = false;

                    for (int i = 0; i < banlistcheck.Length; i++)
                    {
                        if (user == banlistcheck[i].BannedUserID)
                        {
                            alreadyInList = true;
                            break;
                        }

                    }
                    if (!alreadyInList)
                    {

                        if ((estateAccessType & 1) != 0) // All estates
                        {
                            List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                            EstateSettings estateSettings;

                            foreach (int estateID in estateIDs)
                            {
                                if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                                {
                                    EstateBan bitem = new EstateBan();

                                    bitem.BannedUserID = user;
                                    bitem.EstateID = (uint)estateID;
                                    bitem.BannedHostAddress = "0.0.0.0";
                                    bitem.BannedHostIPMask = "0.0.0.0";

                                    estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                    estateSettings.AddBan(bitem);
                                    estateSettings.Save();
                                }
                            }
                        }

                        EstateBan item = new EstateBan();

                        item.BannedUserID = user;
                        item.EstateID = Scene.RegionInfo.EstateSettings.EstateID;
                        item.BannedHostAddress = "0.0.0.0";
                        item.BannedHostIPMask = "0.0.0.0";

                        Scene.RegionInfo.EstateSettings.AddBan(item);
                        Scene.RegionInfo.EstateSettings.Save();

                        TriggerEstateInfoChange();

                        ScenePresence s = Scene.GetScenePresence(user);
                        if (s != null)
                        {
                            if (!s.IsChildAgent)
                            {
                                Scene.TeleportClientHome(user, s.ControllingClient);
                            }
                        }

                    }
                    else
                    {
                        remote_client.SendAlertMessage("User is already on the region ban list");
                    }
                    //m_scene.RegionInfo.regionBanlist.Add(Manager(user);
                    remote_client.SendBannedUserList(invoice, Scene.RegionInfo.EstateSettings.EstateBans, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }
            }
            if ((estateAccessType & 128) != 0) // Ban remove
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, false) || Scene.Permissions.BypassPermissions())
                {
                    EstateBan[] banlistcheck = Scene.RegionInfo.EstateSettings.EstateBans;

                    bool alreadyInList = false;
                    EstateBan listitem = null;

                    for (int i = 0; i < banlistcheck.Length; i++)
                    {
                        if (user == banlistcheck[i].BannedUserID)
                        {
                            alreadyInList = true;
                            listitem = banlistcheck[i];
                            break;
                        }
                    }
                    
                    if (alreadyInList && listitem != null)
                    {
                        if ((estateAccessType & 1) != 0) // All estates
                        {
                            List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                            EstateSettings estateSettings;

                            foreach (int estateID in estateIDs)
                            {
                                if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                                {
                                    estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                    estateSettings.RemoveBan(user);
                                    estateSettings.Save();
                                }
                            }
                        }

                        Scene.RegionInfo.EstateSettings.RemoveBan(listitem.BannedUserID);
                        Scene.RegionInfo.EstateSettings.Save();

                        TriggerEstateInfoChange();
                    }
                    else
                    {
                        remote_client.SendAlertMessage("User is not on the region ban list");
                    }
                    
                    //m_scene.RegionInfo.regionBanlist.Add(Manager(user);
                    remote_client.SendBannedUserList(invoice, Scene.RegionInfo.EstateSettings.EstateBans, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }
            }
            if ((estateAccessType & 256) != 0) // Manager add
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, true) || Scene.Permissions.BypassPermissions())
                {
                    if ((estateAccessType & 1) != 0) // All estates
                    {
                        List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                        EstateSettings estateSettings;

                        foreach (int estateID in estateIDs)
                        {
                            if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                            {
                                estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                estateSettings.AddEstateManager(user);
                                estateSettings.Save();
                            }
                        }
                    }

                    Scene.RegionInfo.EstateSettings.AddEstateManager(user);
                    Scene.RegionInfo.EstateSettings.Save();

                    TriggerEstateInfoChange();
                    remote_client.SendEstateList(invoice, (int)Constants.EstateAccessCodex.EstateManagers, Scene.RegionInfo.EstateSettings.EstateManagers, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }
            }
            if ((estateAccessType & 512) != 0) // Manager remove
            {
                if (Scene.Permissions.CanIssueEstateCommand(remote_client.AgentId, true) || Scene.Permissions.BypassPermissions())
                {
                    if ((estateAccessType & 1) != 0) // All estates
                    {
                        List<int> estateIDs = Scene.EstateDataService.GetEstatesByOwner(Scene.RegionInfo.EstateSettings.EstateOwner);
                        EstateSettings estateSettings;

                        foreach (int estateID in estateIDs)
                        {
                            if (estateID != Scene.RegionInfo.EstateSettings.EstateID)
                            {
                                estateSettings = Scene.EstateDataService.LoadEstateSettings(estateID);
                                estateSettings.RemoveEstateManager(user);
                                estateSettings.Save();
                            }
                        }
                    }

                    Scene.RegionInfo.EstateSettings.RemoveEstateManager(user);
                    Scene.RegionInfo.EstateSettings.Save();

                    TriggerEstateInfoChange();
                    remote_client.SendEstateList(invoice, (int)Constants.EstateAccessCodex.EstateManagers, Scene.RegionInfo.EstateSettings.EstateManagers, Scene.RegionInfo.EstateSettings.EstateID);
                }
                else
                {
                    remote_client.SendAlertMessage("Method EstateAccessDelta Failed, you don't have permissions");
                }
            }
        }

        private void SendSimulatorBlueBoxMessage(
            IClientAPI remote_client, UUID invoice, UUID senderID, UUID sessionID, string senderName, string message)
        {
            IDialogModule dm = Scene.RequestModuleInterface<IDialogModule>();
            
            if (dm != null)
                dm.SendNotificationToUsersInRegion(senderID, senderName, message);
        }

        private void SendEstateBlueBoxMessage(
            IClientAPI remote_client, UUID invoice, UUID senderID, UUID sessionID, string senderName, string message)
        {
            TriggerEstateMessage(senderID, senderName, message);
        }

        private void handleEstateDebugRegionRequest(IClientAPI remote_client, UUID invoice, UUID senderID, bool scripted, bool collisionEvents, bool physics)
        {
            if (physics)
                Scene.RegionInfo.RegionSettings.DisablePhysics = true;
            else
                Scene.RegionInfo.RegionSettings.DisablePhysics = false;

            if (scripted)
                Scene.RegionInfo.RegionSettings.DisableScripts = true;
            else
                Scene.RegionInfo.RegionSettings.DisableScripts = false;

            if (collisionEvents)
                Scene.RegionInfo.RegionSettings.DisableCollisions = true;
            else
                Scene.RegionInfo.RegionSettings.DisableCollisions = false;


            Scene.RegionInfo.RegionSettings.Save();
            TriggerRegionInfoChange();

            Scene.SetSceneCoreDebug(scripted, collisionEvents, physics);
        }

        private void handleEstateTeleportOneUserHomeRequest(IClientAPI remover_client, UUID invoice, UUID senderID, UUID prey)
        {
            if (!Scene.Permissions.CanIssueEstateCommand(remover_client.AgentId, false))
                return;

            if (prey != UUID.Zero)
            {
                ScenePresence s = Scene.GetScenePresence(prey);
                if (s != null)
                {
                    Scene.TeleportClientHome(prey, s.ControllingClient);
                }
            }
        }

        private void handleEstateTeleportAllUsersHomeRequest(IClientAPI remover_client, UUID invoice, UUID senderID)
        {
            if (!Scene.Permissions.CanIssueEstateCommand(remover_client.AgentId, false))
                return;

            Scene.ForEachScenePresence(delegate(ScenePresence sp)
            {
                if (sp.UUID != senderID)
                {
                    ScenePresence p = Scene.GetScenePresence(sp.UUID);
                    // make sure they are still there, we could be working down a long list
                    // Also make sure they are actually in the region
                    if (p != null && !p.IsChildAgent)
                    {
                        Scene.TeleportClientHome(p.UUID, p.ControllingClient);
                    }
                }
            });
        }
        
        private void AbortTerrainXferHandler(IClientAPI remoteClient, ulong XferID)
        {
            if (TerrainUploader != null)
            {
                lock (TerrainUploader)
                {
                    if (XferID == TerrainUploader.XferID)
                    {
                        remoteClient.OnXferReceive -= TerrainUploader.XferReceive;
                        remoteClient.OnAbortXfer -= AbortTerrainXferHandler;
                        TerrainUploader.TerrainUploadDone -= HandleTerrainApplication;

                        TerrainUploader = null;
                        remoteClient.SendAlertMessage("Terrain Upload aborted by the client");
                    }
                }
            }

        }
        private void HandleTerrainApplication(string filename, byte[] terrainData, IClientAPI remoteClient)
        {
            lock (TerrainUploader)
            {
                remoteClient.OnXferReceive -= TerrainUploader.XferReceive;
                remoteClient.OnAbortXfer -= AbortTerrainXferHandler;
                TerrainUploader.TerrainUploadDone -= HandleTerrainApplication;

                TerrainUploader = null;
            }
            remoteClient.SendAlertMessage("Terrain Upload Complete. Loading....");
            ITerrainModule terr = Scene.RequestModuleInterface<ITerrainModule>();

            if (terr != null)
            {
                m_log.Warn("[CLIENT]: Got Request to Send Terrain in region " + Scene.RegionInfo.RegionName);

                try
                {
                    MemoryStream terrainStream = new MemoryStream(terrainData);
                    terr.LoadFromStream(filename, terrainStream);
                    terrainStream.Close();

                    FileInfo x = new FileInfo(filename);
                    remoteClient.SendAlertMessage("Your terrain was loaded as a " + x.Extension + " file. It may take a few moments to appear.");
                }
                catch (IOException e)
                {
                    m_log.ErrorFormat("[TERRAIN]: Error Saving a terrain file uploaded via the estate tools.  It gave us the following error: {0}", e.ToString());
                    remoteClient.SendAlertMessage("There was an IO Exception loading your terrain.  Please check free space.");

                    return;
                }
                catch (SecurityException e)
                {
                    m_log.ErrorFormat("[TERRAIN]: Error Saving a terrain file uploaded via the estate tools.  It gave us the following error: {0}", e.ToString());
                    remoteClient.SendAlertMessage("There was a security Exception loading your terrain.  Please check the security on the simulator drive");

                    return;
                }
                catch (UnauthorizedAccessException e)
                {
                    m_log.ErrorFormat("[TERRAIN]: Error Saving a terrain file uploaded via the estate tools.  It gave us the following error: {0}", e.ToString());
                    remoteClient.SendAlertMessage("There was a security Exception loading your terrain.  Please check the security on the simulator drive");

                    return;
                }
                catch (Exception e)
                {
                    m_log.ErrorFormat("[TERRAIN]: Error loading a terrain file uploaded via the estate tools.  It gave us the following error: {0}", e.ToString());
                    remoteClient.SendAlertMessage("There was a general error loading your terrain.  Please fix the terrain file and try again");
                }
            }
            else
            {
                remoteClient.SendAlertMessage("Unable to apply terrain.  Cannot get an instance of the terrain module");
            }
        }

        private void handleUploadTerrain(IClientAPI remote_client, string clientFileName)
        {
            if (TerrainUploader == null)
            {

                TerrainUploader = new EstateTerrainXferHandler(remote_client, clientFileName);
                lock (TerrainUploader)
                {
                    remote_client.OnXferReceive += TerrainUploader.XferReceive;
                    remote_client.OnAbortXfer += AbortTerrainXferHandler;
                    TerrainUploader.TerrainUploadDone += HandleTerrainApplication;
                }
                TerrainUploader.RequestStartXfer(remote_client);

            }
            else
            {
                remote_client.SendAlertMessage("Another Terrain Upload is in progress.  Please wait your turn!");
            }
        }
        
        private void handleTerrainRequest(IClientAPI remote_client, string clientFileName)
        {
            // Save terrain here
            ITerrainModule terr = Scene.RequestModuleInterface<ITerrainModule>();
            
            if (terr != null)
            {
                m_log.Warn("[CLIENT]: Got Request to Send Terrain in region " + Scene.RegionInfo.RegionName);
                if (File.Exists(Util.dataDir() + "/terrain.raw"))
                {
                    File.Delete(Util.dataDir() + "/terrain.raw");
                }
                terr.SaveToFile(Util.dataDir() + "/terrain.raw");

                FileStream input = new FileStream(Util.dataDir() + "/terrain.raw", FileMode.Open);
                byte[] bdata = new byte[input.Length];
                input.Read(bdata, 0, (int)input.Length);
                remote_client.SendAlertMessage("Terrain file written, starting download...");
                Scene.XferManager.AddNewFile("terrain.raw", bdata);
                // Tell client about it
                m_log.Warn("[CLIENT]: Sending Terrain to " + remote_client.Name);
                remote_client.SendInitiateDownload("terrain.raw", clientFileName);
            }
        }

        private void HandleRegionInfoRequest(IClientAPI remote_client)
        {
           RegionInfoForEstateMenuArgs args = new RegionInfoForEstateMenuArgs();
           args.billableFactor = Scene.RegionInfo.EstateSettings.BillableFactor;
           args.estateID = Scene.RegionInfo.EstateSettings.EstateID;
           args.maxAgents = (byte)Scene.RegionInfo.RegionSettings.AgentLimit;
           args.objectBonusFactor = (float)Scene.RegionInfo.RegionSettings.ObjectBonus;
           args.parentEstateID = Scene.RegionInfo.EstateSettings.ParentEstateID;
           args.pricePerMeter = Scene.RegionInfo.EstateSettings.PricePerMeter;
           args.redirectGridX = Scene.RegionInfo.EstateSettings.RedirectGridX;
           args.redirectGridY = Scene.RegionInfo.EstateSettings.RedirectGridY;
           args.regionFlags = GetRegionFlags();
           args.simAccess = Scene.RegionInfo.AccessLevel;
           args.sunHour = (float)Scene.RegionInfo.RegionSettings.SunPosition;
           args.terrainLowerLimit = (float)Scene.RegionInfo.RegionSettings.TerrainLowerLimit;
           args.terrainRaiseLimit = (float)Scene.RegionInfo.RegionSettings.TerrainRaiseLimit;
           args.useEstateSun = Scene.RegionInfo.RegionSettings.UseEstateSun;
           args.waterHeight = (float)Scene.RegionInfo.RegionSettings.WaterHeight;
           args.simName = Scene.RegionInfo.RegionName;
           args.regionType = Scene.RegionInfo.RegionType;

           remote_client.SendRegionInfoToEstateMenu(args);
        }

        private void HandleEstateCovenantRequest(IClientAPI remote_client)
        {
            remote_client.SendEstateCovenantInformation(Scene.RegionInfo.RegionSettings.Covenant);
        }

        private void HandleLandStatRequest(int parcelID, uint reportType, uint requestFlags, string filter, IClientAPI remoteClient)
        {
            if (!Scene.Permissions.CanIssueEstateCommand(remoteClient.AgentId, false))
                return;

            Dictionary<uint, float> SceneData = new Dictionary<uint,float>();
            List<UUID> uuidNameLookupList = new List<UUID>();

            if (reportType == 1)
            {
                SceneData = Scene.PhysicsScene.GetTopColliders();
            }
            else if (reportType == 0)
            {
                SceneData = Scene.SceneGraph.GetTopScripts();
            }

            List<LandStatReportItem> SceneReport = new List<LandStatReportItem>();
            lock (SceneData)
            {
                foreach (uint obj in SceneData.Keys)
                {
                    SceneObjectPart prt = Scene.GetSceneObjectPart(obj);
                    if (prt != null)
                    {
                        SceneObjectGroup sog = prt.ParentGroup;
                        LandStatReportItem lsri = new LandStatReportItem();
                        lsri.LocationX = sog.AbsolutePosition.X;
                        lsri.LocationY = sog.AbsolutePosition.Y;
                        lsri.LocationZ = sog.AbsolutePosition.Z;
                        lsri.Score = SceneData[obj];
                        lsri.TaskID = sog.UUID;
                        lsri.TaskLocalID = sog.LocalId;
                        lsri.TaskName = sog.GetPartName(obj);
                        lsri.OwnerName = "waiting";
                        lock (uuidNameLookupList)
                            uuidNameLookupList.Add(sog.OwnerID);

                        if (filter.Length != 0)
                        {
                            if ((lsri.OwnerName.Contains(filter) || lsri.TaskName.Contains(filter)))
                            {
                            }
                            else
                            {
                                continue;
                            }
                        }

                        SceneReport.Add(lsri);
                    }
                }
            }

            remoteClient.SendLandStatReply(reportType, requestFlags, (uint)SceneReport.Count,SceneReport.ToArray());

            if (uuidNameLookupList.Count > 0)
                LookupUUID(uuidNameLookupList);
        }

        private static void LookupUUIDSCompleted(IAsyncResult iar)
        {
            LookupUUIDS icon = (LookupUUIDS)iar.AsyncState;
            icon.EndInvoke(iar);
        }
        
        private void LookupUUID(List<UUID> uuidLst)
        {
            LookupUUIDS d = LookupUUIDsAsync;

            d.BeginInvoke(uuidLst,
                          LookupUUIDSCompleted,
                          d);
        }
        
        private void LookupUUIDsAsync(List<UUID> uuidLst)
        {
            UUID[] uuidarr;

            lock (uuidLst)
            {
                uuidarr = uuidLst.ToArray();
            }

            for (int i = 0; i < uuidarr.Length; i++)
            {
                // string lookupname = m_scene.CommsManager.UUIDNameRequestString(uuidarr[i]);

                IUserManagement userManager = Scene.RequestModuleInterface<IUserManagement>();
                if (userManager != null)
                    userManager.GetUserName(uuidarr[i]);
                
                // we drop it.  It gets cached though...  so we're ready for the next request.
                // diva commnent 11/21/2010: uh?!? wft?
                // justincc comment 21/01/2011: A side effect of userManager.GetUserName() I presume.
            }
        }
        #endregion

        #region Outgoing Packets

        public void sendRegionInfoPacketToAll()
        {
            Scene.ForEachScenePresence(delegate(ScenePresence sp)
            {
                if (!sp.IsChildAgent)
                    HandleRegionInfoRequest(sp.ControllingClient);
            });
        }

        public void sendRegionHandshake(IClientAPI remoteClient)
        {
            RegionHandshakeArgs args = new RegionHandshakeArgs();

            args.isEstateManager = Scene.RegionInfo.EstateSettings.IsEstateManager(remoteClient.AgentId);
            if (Scene.RegionInfo.EstateSettings.EstateOwner != UUID.Zero && Scene.RegionInfo.EstateSettings.EstateOwner == remoteClient.AgentId)
                args.isEstateManager = true;

            args.billableFactor = Scene.RegionInfo.EstateSettings.BillableFactor;
            args.terrainStartHeight0 = (float)Scene.RegionInfo.RegionSettings.Elevation1SW;
            args.terrainHeightRange0 = (float)Scene.RegionInfo.RegionSettings.Elevation2SW;
            args.terrainStartHeight1 = (float)Scene.RegionInfo.RegionSettings.Elevation1NW;
            args.terrainHeightRange1 = (float)Scene.RegionInfo.RegionSettings.Elevation2NW;
            args.terrainStartHeight2 = (float)Scene.RegionInfo.RegionSettings.Elevation1SE;
            args.terrainHeightRange2 = (float)Scene.RegionInfo.RegionSettings.Elevation2SE;
            args.terrainStartHeight3 = (float)Scene.RegionInfo.RegionSettings.Elevation1NE;
            args.terrainHeightRange3 = (float)Scene.RegionInfo.RegionSettings.Elevation2NE;
            args.simAccess = Scene.RegionInfo.AccessLevel;
            args.waterHeight = (float)Scene.RegionInfo.RegionSettings.WaterHeight;
            args.regionFlags = GetRegionFlags();
            args.regionName = Scene.RegionInfo.RegionName;
            args.SimOwner = Scene.RegionInfo.EstateSettings.EstateOwner;

            args.terrainBase0 = UUID.Zero;
            args.terrainBase1 = UUID.Zero;
            args.terrainBase2 = UUID.Zero;
            args.terrainBase3 = UUID.Zero;
            args.terrainDetail0 = Scene.RegionInfo.RegionSettings.TerrainTexture1;
            args.terrainDetail1 = Scene.RegionInfo.RegionSettings.TerrainTexture2;
            args.terrainDetail2 = Scene.RegionInfo.RegionSettings.TerrainTexture3;
            args.terrainDetail3 = Scene.RegionInfo.RegionSettings.TerrainTexture4;

            m_log.DebugFormat("[ESTATE MANAGEMENT MODULE]: Sending terrain texture 1 {0} for region {1}", args.terrainDetail0, Scene.RegionInfo.RegionName);
            m_log.DebugFormat("[ESTATE MANAGEMENT MODULE]: Sending terrain texture 2 {0} for region {1}", args.terrainDetail1, Scene.RegionInfo.RegionName);
            m_log.DebugFormat("[ESTATE MANAGEMENT MODULE]: Sending terrain texture 3 {0} for region {1}", args.terrainDetail2, Scene.RegionInfo.RegionName);
            m_log.DebugFormat("[ESTATE MANAGEMENT MODULE]: Sending terrain texture 4 {0} for region {1}", args.terrainDetail3, Scene.RegionInfo.RegionName);

            remoteClient.SendRegionHandshake(Scene.RegionInfo,args);
        }

        public void sendRegionHandshakeToAll()
        {
            Scene.ForEachClient(sendRegionHandshake);
        }

        public void handleEstateChangeInfo(IClientAPI remoteClient, UUID invoice, UUID senderID, UInt32 parms1, UInt32 parms2)
        {
            if (parms2 == 0)
            {
                Scene.RegionInfo.EstateSettings.UseGlobalTime = true;
                Scene.RegionInfo.EstateSettings.SunPosition = 0.0;
            }
            else
            {
                Scene.RegionInfo.EstateSettings.UseGlobalTime = false;
                Scene.RegionInfo.EstateSettings.SunPosition = (parms2 - 0x1800)/1024.0;
            }

            if ((parms1 & 0x00000010) != 0)
                Scene.RegionInfo.EstateSettings.FixedSun = true;
            else
                Scene.RegionInfo.EstateSettings.FixedSun = false;

            if ((parms1 & 0x00008000) != 0)
                Scene.RegionInfo.EstateSettings.PublicAccess = true;
            else
                Scene.RegionInfo.EstateSettings.PublicAccess = false;

            if ((parms1 & 0x10000000) != 0)
                Scene.RegionInfo.EstateSettings.AllowVoice = true;
            else
                Scene.RegionInfo.EstateSettings.AllowVoice = false;

            if ((parms1 & 0x00100000) != 0)
                Scene.RegionInfo.EstateSettings.AllowDirectTeleport = true;
            else
                Scene.RegionInfo.EstateSettings.AllowDirectTeleport = false;

            if ((parms1 & 0x00800000) != 0)
                Scene.RegionInfo.EstateSettings.DenyAnonymous = true;
            else
                Scene.RegionInfo.EstateSettings.DenyAnonymous = false;

            if ((parms1 & 0x01000000) != 0)
                Scene.RegionInfo.EstateSettings.DenyIdentified = true;
            else
                Scene.RegionInfo.EstateSettings.DenyIdentified = false;

            if ((parms1 & 0x02000000) != 0)
                Scene.RegionInfo.EstateSettings.DenyTransacted = true;
            else
                Scene.RegionInfo.EstateSettings.DenyTransacted = false;

            if ((parms1 & 0x40000000) != 0)
                Scene.RegionInfo.EstateSettings.DenyMinors = true;
            else
                Scene.RegionInfo.EstateSettings.DenyMinors = false;

            Scene.RegionInfo.EstateSettings.Save();
            TriggerEstateInfoChange();

            Scene.TriggerEstateSunUpdate();

            sendDetailedEstateData(remoteClient, invoice);
        }

        #endregion

        #region IRegionModule Members
        
        public string Name { get { return "EstateManagementModule"; } }
        
        public Type ReplaceableInterface { get { return null; } }        

        public void Initialise(IConfigSource source) {}
        
        public void AddRegion(Scene scene)
        {
            Scene = scene;
            Scene.RegisterModuleInterface<IEstateModule>(this);
            Scene.EventManager.OnNewClient += EventManager_OnNewClient;
            Scene.EventManager.OnRequestChangeWaterHeight += changeWaterHeight;
            
            m_commands = new EstateManagementCommands(this);
            m_commands.Initialise();
        }
        
        public void RemoveRegion(Scene scene) {}            
        
        public void RegionLoaded(Scene scene)
        {
            // Sets up the sun module based no the saved Estate and Region Settings
            // DO NOT REMOVE or the sun will stop working
            scene.TriggerEstateSunUpdate();
            
            UserManager = scene.RequestModuleInterface<IUserManagement>();            
        }

        public void Close() 
        {
            m_commands.Close();
        }

        #endregion

        #region Other Functions

        public void changeWaterHeight(float height)
        {
            setRegionTerrainSettings(height,
                    (float)Scene.RegionInfo.RegionSettings.TerrainRaiseLimit,
                    (float)Scene.RegionInfo.RegionSettings.TerrainLowerLimit,
                    Scene.RegionInfo.RegionSettings.UseEstateSun,
                    Scene.RegionInfo.RegionSettings.FixedSun,
                    (float)Scene.RegionInfo.RegionSettings.SunPosition,
                    Scene.RegionInfo.EstateSettings.UseGlobalTime,
                    Scene.RegionInfo.EstateSettings.FixedSun,
                    (float)Scene.RegionInfo.EstateSettings.SunPosition);

            sendRegionInfoPacketToAll();
        }

        #endregion

        private void EventManager_OnNewClient(IClientAPI client)
        {
            client.OnDetailedEstateDataRequest += sendDetailedEstateData;
            client.OnSetEstateFlagsRequest += estateSetRegionInfoHandler;
//            client.OnSetEstateTerrainBaseTexture += setEstateTerrainBaseTexture;
            client.OnSetEstateTerrainDetailTexture += setEstateTerrainBaseTexture;
            client.OnSetEstateTerrainTextureHeights += setEstateTerrainTextureHeights;
            client.OnCommitEstateTerrainTextureRequest += handleCommitEstateTerrainTextureRequest;
            client.OnSetRegionTerrainSettings += setRegionTerrainSettings;
            client.OnEstateRestartSimRequest += handleEstateRestartSimRequest;
            client.OnEstateChangeCovenantRequest += handleChangeEstateCovenantRequest;
            client.OnEstateChangeInfo += handleEstateChangeInfo;
            client.OnUpdateEstateAccessDeltaRequest += handleEstateAccessDeltaRequest;
            client.OnSimulatorBlueBoxMessageRequest += SendSimulatorBlueBoxMessage;
            client.OnEstateBlueBoxMessageRequest += SendEstateBlueBoxMessage;
            client.OnEstateDebugRegionRequest += handleEstateDebugRegionRequest;
            client.OnEstateTeleportOneUserHomeRequest += handleEstateTeleportOneUserHomeRequest;
            client.OnEstateTeleportAllUsersHomeRequest += handleEstateTeleportAllUsersHomeRequest;
            client.OnRequestTerrain += handleTerrainRequest;
            client.OnUploadTerrain += handleUploadTerrain;

            client.OnRegionInfoRequest += HandleRegionInfoRequest;
            client.OnEstateCovenantRequest += HandleEstateCovenantRequest;
            client.OnLandStatRequest += HandleLandStatRequest;
            sendRegionHandshake(client);
        }

        public uint GetRegionFlags()
        {
            RegionFlags flags = RegionFlags.None;

            // Fully implemented
            //
            if (Scene.RegionInfo.RegionSettings.AllowDamage)
                flags |= RegionFlags.AllowDamage;
            if (Scene.RegionInfo.RegionSettings.BlockTerraform)
                flags |= RegionFlags.BlockTerraform;
            if (!Scene.RegionInfo.RegionSettings.AllowLandResell)
                flags |= RegionFlags.BlockLandResell;
            if (Scene.RegionInfo.RegionSettings.DisableCollisions)
                flags |= RegionFlags.SkipCollisions;
            if (Scene.RegionInfo.RegionSettings.DisableScripts)
                flags |= RegionFlags.SkipScripts;
            if (Scene.RegionInfo.RegionSettings.DisablePhysics)
                flags |= RegionFlags.SkipPhysics;
            if (Scene.RegionInfo.RegionSettings.BlockFly)
                flags |= RegionFlags.NoFly;
            if (Scene.RegionInfo.RegionSettings.RestrictPushing)
                flags |= RegionFlags.RestrictPushObject;
            if (Scene.RegionInfo.RegionSettings.AllowLandJoinDivide)
                flags |= RegionFlags.AllowParcelChanges;
            if (Scene.RegionInfo.RegionSettings.BlockShowInSearch)
                flags |= RegionFlags.BlockParcelSearch;

            if (Scene.RegionInfo.RegionSettings.FixedSun)
                flags |= RegionFlags.SunFixed;
            if (Scene.RegionInfo.RegionSettings.Sandbox)
                flags |= RegionFlags.Sandbox;
            if (Scene.RegionInfo.EstateSettings.AllowVoice)
                flags |= RegionFlags.AllowVoice;

            // Fudge these to always on, so the menu options activate
            //
            flags |= RegionFlags.AllowLandmark;
            flags |= RegionFlags.AllowSetHome;

            // TODO: SkipUpdateInterestList

            // Omitted
            //
            // Omitted: NullLayer (what is that?)
            // Omitted: SkipAgentAction (what does it do?)

            return (uint)flags;
        }

        public uint GetEstateFlags()
        {
            RegionFlags flags = RegionFlags.None;

            if (Scene.RegionInfo.EstateSettings.FixedSun)
                flags |= RegionFlags.SunFixed;
            if (Scene.RegionInfo.EstateSettings.PublicAccess)
                flags |= (RegionFlags.PublicAllowed |
                          RegionFlags.ExternallyVisible);
            if (Scene.RegionInfo.EstateSettings.AllowVoice)
                flags |= RegionFlags.AllowVoice;
            if (Scene.RegionInfo.EstateSettings.AllowDirectTeleport)
                flags |= RegionFlags.AllowDirectTeleport;
            if (Scene.RegionInfo.EstateSettings.DenyAnonymous)
                flags |= RegionFlags.DenyAnonymous;
            if (Scene.RegionInfo.EstateSettings.DenyIdentified)
                flags |= RegionFlags.DenyIdentified;
            if (Scene.RegionInfo.EstateSettings.DenyTransacted)
                flags |= RegionFlags.DenyTransacted;
            if (Scene.RegionInfo.EstateSettings.AbuseEmailToEstateOwner)
                flags |= RegionFlags.AbuseEmailToEstateOwner;
            if (Scene.RegionInfo.EstateSettings.BlockDwell)
                flags |= RegionFlags.BlockDwell;
            if (Scene.RegionInfo.EstateSettings.EstateSkipScripts)
                flags |= RegionFlags.EstateSkipScripts;
            if (Scene.RegionInfo.EstateSettings.ResetHomeOnTeleport)
                flags |= RegionFlags.ResetHomeOnTeleport;
            if (Scene.RegionInfo.EstateSettings.TaxFree)
                flags |= RegionFlags.TaxFree;
            if (Scene.RegionInfo.EstateSettings.DenyMinors)
                flags |= (RegionFlags)(1 << 30);

            return (uint)flags;
        }

        public bool IsManager(UUID avatarID)
        {
            if (avatarID == Scene.RegionInfo.EstateSettings.EstateOwner)
                return true;

            List<UUID> ems = new List<UUID>(Scene.RegionInfo.EstateSettings.EstateManagers);
            if (ems.Contains(avatarID))
                return true;

            return false;
        }

        public void TriggerRegionInfoChange()
        {
            ChangeDelegate change = OnRegionInfoChange;

            if (change != null)
                change(Scene.RegionInfo.RegionID);
        }

        public void TriggerEstateInfoChange()
        {
            ChangeDelegate change = OnEstateInfoChange;

            if (change != null)
                change(Scene.RegionInfo.RegionID);
        }

        public void TriggerEstateMessage(UUID fromID, string fromName, string message)
        {
            MessageDelegate onmessage = OnEstateMessage;

            if (onmessage != null)
                onmessage(Scene.RegionInfo.RegionID, fromID, fromName, message);
        }
    }
}
