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
using System.Reflection;
using System.Threading;
using log4net;
using Nini.Config;
using OpenMetaverse;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;
using OpenSim.Framework;
using Timer=System.Timers.Timer;
using OpenSim.Services.Interfaces;

namespace OpenSim.Region.OptionalModules.World.NPC
{
    public class NPCModule : IRegionModule, INPCModule
    {
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private Dictionary<UUID, NPCAvatar> m_avatars = new Dictionary<UUID, NPCAvatar>();

        public void Initialise(Scene scene, IConfigSource source)
        {
            IConfig config = source.Configs["NPC"];

            if (config != null && config.GetBoolean("Enabled", false))
            {
                scene.RegisterModuleInterface<INPCModule>(this);
            }
        }

        public bool IsNPC(UUID agentId, Scene scene)
        {
            // FIXME: This implementation could not just use the ScenePresence.PresenceType (and callers could inspect
            // that directly).
            ScenePresence sp = scene.GetScenePresence(agentId);
            if (sp == null || sp.IsChildAgent)
                return false;

            lock (m_avatars)
                return m_avatars.ContainsKey(agentId);
        }

        public bool SetNPCAppearance(UUID agentId, AvatarAppearance appearance, Scene scene)
        {
            ScenePresence sp = scene.GetScenePresence(agentId);
            if (sp == null || sp.IsChildAgent)
                return false;

            lock (m_avatars)
                if (!m_avatars.ContainsKey(agentId))
                    return false;

            scene.AttachmentsModule.DeleteAttachmentsFromScene(sp, false);

            AvatarAppearance npcAppearance = new AvatarAppearance(appearance, true);
            sp.Appearance = npcAppearance;
            scene.AttachmentsModule.RezAttachments(sp);

            IAvatarFactoryModule module = scene.RequestModuleInterface<IAvatarFactoryModule>();
            module.SendAppearance(sp.UUID);

            return true;
        }

        public UUID CreateNPC(
            string firstname, string lastname, Vector3 position, UUID owner, Scene scene, AvatarAppearance appearance)
        {
            NPCAvatar npcAvatar = new NPCAvatar(firstname, lastname, position, owner, scene);
            npcAvatar.CircuitCode = (uint)Util.RandomClass.Next(0, int.MaxValue);

            m_log.DebugFormat(
                "[NPC MODULE]: Creating NPC {0} {1} {2} at {3} in {4}",
                firstname, lastname, npcAvatar.AgentId, position, scene.RegionInfo.RegionName);

            AgentCircuitData acd = new AgentCircuitData();
            acd.AgentID = npcAvatar.AgentId;
            acd.firstname = firstname;
            acd.lastname = lastname;
            acd.ServiceURLs = new Dictionary<string, object>();

            AvatarAppearance npcAppearance = new AvatarAppearance(appearance, true);
            acd.Appearance = npcAppearance;

//            for (int i = 0; i < acd.Appearance.Texture.FaceTextures.Length; i++)
//            {
//                m_log.DebugFormat(
//                    "[NPC MODULE]: NPC avatar {0} has texture id {1} : {2}",
//                    acd.AgentID, i, acd.Appearance.Texture.FaceTextures[i]);
//            }

            scene.AuthenticateHandler.AddNewCircuit(npcAvatar.CircuitCode, acd);
            scene.AddNewClient(npcAvatar, PresenceType.Npc);

            ScenePresence sp;
            if (scene.TryGetScenePresence(npcAvatar.AgentId, out sp))
            {
                m_log.DebugFormat(
                    "[NPC MODULE]: Successfully retrieved scene presence for NPC {0} {1}", sp.Name, sp.UUID);

                sp.CompleteMovement(npcAvatar, false);
            }
            else
            {
                m_log.WarnFormat("[NPC MODULE]: Could not find scene presence for NPC {0} {1}", sp.Name, sp.UUID);
            }

            lock (m_avatars)
                m_avatars.Add(npcAvatar.AgentId, npcAvatar);

            m_log.DebugFormat("[NPC MODULE]: Created NPC with id {0}", npcAvatar.AgentId);

            return npcAvatar.AgentId;
        }

        public bool MoveToTarget(UUID agentID, Scene scene, Vector3 pos, bool noFly, bool landAtTarget)
        {
            lock (m_avatars)
            {
                if (m_avatars.ContainsKey(agentID))
                {
                    ScenePresence sp;
                    scene.TryGetScenePresence(agentID, out sp);

                    m_log.DebugFormat(
                        "[NPC MODULE]: Moving {0} to {1} in {2}, noFly {3}, landAtTarget {4}",
                        sp.Name, pos, scene.RegionInfo.RegionName, noFly, landAtTarget);

                    sp.MoveToTarget(pos, noFly, landAtTarget);

                    return true;
                }
            }

            return false;
        }

        public bool StopMoveToTarget(UUID agentID, Scene scene)
        {
            lock (m_avatars)
            {
                if (m_avatars.ContainsKey(agentID))
                {
                    ScenePresence sp;
                    scene.TryGetScenePresence(agentID, out sp);

                    sp.Velocity = Vector3.Zero;
                    sp.ResetMoveToTarget();

                    return true;
                }
            }

            return false;
        }

        public bool Say(UUID agentID, Scene scene, string text)
        {
            lock (m_avatars)
            {
                if (m_avatars.ContainsKey(agentID))
                {
                    ScenePresence sp;
                    scene.TryGetScenePresence(agentID, out sp);

                    m_avatars[agentID].Say(text);

                    return true;
                }
            }

            return false;
        }

        public bool Sit(UUID agentID, UUID partID, Scene scene)
        {
            lock (m_avatars)
            {
                if (m_avatars.ContainsKey(agentID))
                {
                    ScenePresence sp;
                    scene.TryGetScenePresence(agentID, out sp);
                    sp.HandleAgentRequestSit(m_avatars[agentID], agentID, partID, Vector3.Zero);
//                    sp.HandleAgentSit(m_avatars[agentID], agentID);

                    return true;
                }
            }

            return false;
        }

        public bool Stand(UUID agentID, Scene scene)
        {
            lock (m_avatars)
            {
                if (m_avatars.ContainsKey(agentID))
                {
                    ScenePresence sp;
                    scene.TryGetScenePresence(agentID, out sp);
                    sp.StandUp();

                    return true;
                }
            }

            return false;
        }

        public UUID GetOwner(UUID agentID)
        {
            lock (m_avatars)
            {
                NPCAvatar av;
                if (m_avatars.TryGetValue(agentID, out av))
                {
                    return av.OwnerID;
                }
            }

            return UUID.Zero;
        }

        public bool DeleteNPC(UUID agentID, UUID callerID, Scene scene)
        {
            lock (m_avatars)
            {
                NPCAvatar av;
                if (m_avatars.TryGetValue(agentID, out av))
                {
                    if (av.OwnerID != UUID.Zero && callerID != UUID.Zero && av.OwnerID != callerID)
                        return false;

                    scene.RemoveClient(agentID, false);
                    m_avatars.Remove(agentID);

                    return true;
                }
            }

            return false;
        }

        public void PostInitialise()
        {
        }

        public void Close()
        {
        }

        public string Name
        {
            get { return "NPCModule"; }
        }

        public bool IsSharedModule
        {
            get { return true; }
        }
    }
}
