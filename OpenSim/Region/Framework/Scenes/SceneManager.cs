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
using OpenMetaverse;
using log4net;
using OpenSim.Framework;
using OpenSim.Region.Framework.Interfaces;

namespace OpenSim.Region.Framework.Scenes
{
    public delegate void RestartSim(RegionInfo thisregion);

    /// <summary>
    /// Manager for adding, closing and restarting scenes.
    /// </summary>
    public class SceneManager
    {
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public event RestartSim OnRestartSim;

        private static SceneManager m_instance = null;
        public static SceneManager Instance
        { 
            get { return m_instance; } 
        }

        private readonly List<Scene> m_localScenes = new List<Scene>();
        private Scene m_currentScene = null;

        public List<Scene> Scenes
        {
            get { return new List<Scene>(m_localScenes); }
        }

        public Scene CurrentScene
        {
            get { return m_currentScene; }
        }

        public Scene CurrentOrFirstScene
        {
            get
            {
                if (m_currentScene == null)
                {
                    lock (m_localScenes)
                    {
                        if (m_localScenes.Count > 0)
                            return m_localScenes[0];
                        else
                            return null;
                    }
                }
                else
                {
                    return m_currentScene;
                }
            }
        }

        public SceneManager()
        {
            m_instance = this;
            m_localScenes = new List<Scene>();
        }

        public void Close()
        {
            // collect known shared modules in sharedModules
            Dictionary<string, IRegionModule> sharedModules = new Dictionary<string, IRegionModule>();

            lock (m_localScenes)
            {
                for (int i = 0; i < m_localScenes.Count; i++)
                {
                    // extract known shared modules from scene
                    foreach (string k in m_localScenes[i].Modules.Keys)
                    {
                        if (m_localScenes[i].Modules[k].IsSharedModule &&
                            !sharedModules.ContainsKey(k))
                            sharedModules[k] = m_localScenes[i].Modules[k];
                    }
                    // close scene/region
                    m_localScenes[i].Close();
                }
            }

            // all regions/scenes are now closed, we can now safely
            // close all shared modules
            foreach (IRegionModule mod in sharedModules.Values)
            {
                mod.Close();
            }
        }

        public void Close(Scene cscene)
        {
            lock (m_localScenes)
            {
                if (m_localScenes.Contains(cscene))
                {
                    for (int i = 0; i < m_localScenes.Count; i++)
                    {
                        if (m_localScenes[i].Equals(cscene))
                        {
                            m_localScenes[i].Close();
                        }
                    }
                }
            }
        }

        public void Add(Scene scene)
        {
            scene.OnRestart += HandleRestart;

            lock (m_localScenes)
                m_localScenes.Add(scene);
        }

        public void HandleRestart(RegionInfo rdata)
        {
            m_log.Error("[SCENEMANAGER]: Got Restart message for region:" + rdata.RegionName + " Sending up to main");
            int RegionSceneElement = -1;

            lock (m_localScenes)
            {
                for (int i = 0; i < m_localScenes.Count; i++)
                {
                    if (rdata.RegionName == m_localScenes[i].RegionInfo.RegionName)
                    {
                        RegionSceneElement = i;
                    }
                }

                // Now we make sure the region is no longer known about by the SceneManager
                // Prevents duplicates.

                if (RegionSceneElement >= 0)
                {
                    m_localScenes.RemoveAt(RegionSceneElement);
                }
            }

            // Send signal to main that we're restarting this sim.
            OnRestartSim(rdata);
        }

        public void SendSimOnlineNotification(ulong regionHandle)
        {
            RegionInfo Result = null;

            lock (m_localScenes)
            {
                for (int i = 0; i < m_localScenes.Count; i++)
                {
                    if (m_localScenes[i].RegionInfo.RegionHandle == regionHandle)
                    {
                        // Inform other regions to tell their avatar about me
                        Result = m_localScenes[i].RegionInfo;
                    }
                }

                if (Result != null)
                {
                    for (int i = 0; i < m_localScenes.Count; i++)
                    {
                        if (m_localScenes[i].RegionInfo.RegionHandle != regionHandle)
                        {
                            // Inform other regions to tell their avatar about me
                            //m_localScenes[i].OtherRegionUp(Result);
                        }
                    }
                }
                else
                {
                    m_log.Error("[REGION]: Unable to notify Other regions of this Region coming up");
                }
            }
        }

        /// <summary>
        /// Save the prims in the current scene to an xml file in OpenSimulator's original 'xml' format
        /// </summary>
        /// <param name="filename"></param>
        public void SaveCurrentSceneToXml(string filename)
        {
            IRegionSerialiserModule serialiser = CurrentOrFirstScene.RequestModuleInterface<IRegionSerialiserModule>();
            if (serialiser != null)
                serialiser.SavePrimsToXml(CurrentOrFirstScene, filename);
        }

        /// <summary>
        /// Load an xml file of prims in OpenSimulator's original 'xml' file format to the current scene
        /// </summary>
        /// <param name="filename"></param>
        /// <param name="generateNewIDs"></param>
        /// <param name="loadOffset"></param>
        public void LoadCurrentSceneFromXml(string filename, bool generateNewIDs, Vector3 loadOffset)
        {
            IRegionSerialiserModule serialiser = CurrentOrFirstScene.RequestModuleInterface<IRegionSerialiserModule>();
            if (serialiser != null)
                serialiser.LoadPrimsFromXml(CurrentOrFirstScene, filename, generateNewIDs, loadOffset);
        }

        /// <summary>
        /// Save the prims in the current scene to an xml file in OpenSimulator's current 'xml2' format
        /// </summary>
        /// <param name="filename"></param>
        public void SaveCurrentSceneToXml2(string filename)
        {
            IRegionSerialiserModule serialiser = CurrentOrFirstScene.RequestModuleInterface<IRegionSerialiserModule>();
            if (serialiser != null)
                serialiser.SavePrimsToXml2(CurrentOrFirstScene, filename);
        }

        public void SaveNamedPrimsToXml2(string primName, string filename)
        {
            IRegionSerialiserModule serialiser = CurrentOrFirstScene.RequestModuleInterface<IRegionSerialiserModule>();
            if (serialiser != null)
                serialiser.SaveNamedPrimsToXml2(CurrentOrFirstScene, primName, filename);
        }

        /// <summary>
        /// Load an xml file of prims in OpenSimulator's current 'xml2' file format to the current scene
        /// </summary>
        public void LoadCurrentSceneFromXml2(string filename)
        {
            IRegionSerialiserModule serialiser = CurrentOrFirstScene.RequestModuleInterface<IRegionSerialiserModule>();
            if (serialiser != null)
                serialiser.LoadPrimsFromXml2(CurrentOrFirstScene, filename);
        }

        /// <summary>
        /// Save the current scene to an OpenSimulator archive.  This archive will eventually include the prim's assets
        /// as well as the details of the prims themselves.
        /// </summary>
        /// <param name="cmdparams"></param>
        public void SaveCurrentSceneToArchive(string[] cmdparams)
        {
            IRegionArchiverModule archiver = CurrentOrFirstScene.RequestModuleInterface<IRegionArchiverModule>();
            if (archiver != null)
                archiver.HandleSaveOarConsoleCommand(string.Empty, cmdparams);
        }

        /// <summary>
        /// Load an OpenSim archive into the current scene.  This will load both the shapes of the prims and upload
        /// their assets to the asset service.
        /// </summary>
        /// <param name="cmdparams"></param>
        public void LoadArchiveToCurrentScene(string[] cmdparams)
        {
            IRegionArchiverModule archiver = CurrentOrFirstScene.RequestModuleInterface<IRegionArchiverModule>();
            if (archiver != null)
                archiver.HandleLoadOarConsoleCommand(string.Empty, cmdparams);
        }

        public string SaveCurrentSceneMapToXmlString()
        {
            return CurrentOrFirstScene.Heightmap.SaveToXmlString();
        }

        public void LoadCurrenSceneMapFromXmlString(string mapData)
        {
            CurrentOrFirstScene.Heightmap.LoadFromXmlString(mapData);
        }

        public void SendCommandToPluginModules(string[] cmdparams)
        {
            ForEachCurrentScene(delegate(Scene scene) { scene.SendCommandToPlugins(cmdparams); });
        }

        public void SetBypassPermissionsOnCurrentScene(bool bypassPermissions)
        {
            ForEachCurrentScene(delegate(Scene scene) { scene.Permissions.SetBypassPermissions(bypassPermissions); });
        }

        private void ForEachCurrentScene(Action<Scene> func)
        {
            if (m_currentScene == null)
            {
                lock (m_localScenes)
                    m_localScenes.ForEach(func);
            }
            else
            {
                func(m_currentScene);
            }
        }

        public void RestartCurrentScene()
        {
            ForEachCurrentScene(delegate(Scene scene) { scene.RestartNow(); });
        }

        public void BackupCurrentScene()
        {
            ForEachCurrentScene(delegate(Scene scene) { scene.Backup(true); });
        }

        public bool TrySetCurrentScene(string regionName)
        {
            if ((String.Compare(regionName, "root") == 0) 
                || (String.Compare(regionName, "..") == 0)
                || (String.Compare(regionName, "/") == 0))
            {
                m_currentScene = null;
                return true;
            }
            else
            {
                lock (m_localScenes)
                {
                    foreach (Scene scene in m_localScenes)
                    {
                        if (String.Compare(scene.RegionInfo.RegionName, regionName, true) == 0)
                        {
                            m_currentScene = scene;
                            return true;
                        }
                    }
                }

                return false;
            }
        }

        public bool TrySetCurrentScene(UUID regionID)
        {
            m_log.Debug("Searching for Region: '" + regionID + "'");

            lock (m_localScenes)
            {
                foreach (Scene scene in m_localScenes)
                {
                    if (scene.RegionInfo.RegionID == regionID)
                    {
                        m_currentScene = scene;
                        return true;
                    }
                }
            }

            return false;
        }

        public bool TryGetScene(string regionName, out Scene scene)
        {
            lock (m_localScenes)
            {
                foreach (Scene mscene in m_localScenes)
                {
                    if (String.Compare(mscene.RegionInfo.RegionName, regionName, true) == 0)
                    {
                        scene = mscene;
                        return true;
                    }
                }
            }

            scene = null;
            return false;
        }

        public bool TryGetScene(UUID regionID, out Scene scene)
        {
            lock (m_localScenes)
            {
                foreach (Scene mscene in m_localScenes)
                {
                    if (mscene.RegionInfo.RegionID == regionID)
                    {
                        scene = mscene;
                        return true;
                    }
                }
            }
            
            scene = null;
            return false;
        }

        public bool TryGetScene(uint locX, uint locY, out Scene scene)
        {
            lock (m_localScenes)
            {
                foreach (Scene mscene in m_localScenes)
                {
                    if (mscene.RegionInfo.RegionLocX == locX &&
                        mscene.RegionInfo.RegionLocY == locY)
                    {
                        scene = mscene;
                        return true;
                    }
                }
            }
            
            scene = null;
            return false;
        }

        public bool TryGetScene(IPEndPoint ipEndPoint, out Scene scene)
        {
            lock (m_localScenes)
            {
                foreach (Scene mscene in m_localScenes)
                {
                    if ((mscene.RegionInfo.InternalEndPoint.Equals(ipEndPoint.Address)) &&
                        (mscene.RegionInfo.InternalEndPoint.Port == ipEndPoint.Port))
                    {
                        scene = mscene;
                        return true;
                    }
                }
            }
            
            scene = null;
            return false;
        }

        /// <summary>
        /// Set the debug packet level on each current scene.  This level governs which packets are printed out to the
        /// console.
        /// </summary>
        /// <param name="newDebug"></param>
        /// <param name="name">Name of avatar to debug</param>
        public void SetDebugPacketLevelOnCurrentScene(int newDebug, string name)
        {
            ForEachCurrentScene(scene =>
                scene.ForEachScenePresence(sp =>
                {
                    if (name == null || sp.Name == name)
                    {
                        m_log.DebugFormat(
                            "Packet debug for {0} ({1}) set to {2}",
                            sp.Name, sp.IsChildAgent ? "child" : "root", newDebug);

                        sp.ControllingClient.DebugPacketLevel = newDebug;
                    }
                })
            );
        }

        public List<ScenePresence> GetCurrentSceneAvatars()
        {
            List<ScenePresence> avatars = new List<ScenePresence>();

            ForEachCurrentScene(
                delegate(Scene scene)
                {
                    scene.ForEachRootScenePresence(delegate(ScenePresence scenePresence)
                    {
                        avatars.Add(scenePresence);
                    });
                }
            );

            return avatars;
        }

        public List<ScenePresence> GetCurrentScenePresences()
        {
            List<ScenePresence> presences = new List<ScenePresence>();

            ForEachCurrentScene(delegate(Scene scene)
            {
                scene.ForEachScenePresence(delegate(ScenePresence sp)
                {
                    presences.Add(sp);
                });
            });

            return presences;
        }

        public RegionInfo GetRegionInfo(UUID regionID)
        {
            lock (m_localScenes)
            {
                foreach (Scene scene in m_localScenes)
                {
                    if (scene.RegionInfo.RegionID == regionID)
                    {
                        return scene.RegionInfo;
                    }
                }
            }

            return null;
        }

        public void ForceCurrentSceneClientUpdate()
        {
            ForEachCurrentScene(delegate(Scene scene) { scene.ForceClientUpdate(); });
        }

        public void HandleEditCommandOnCurrentScene(string[] cmdparams)
        {
            ForEachCurrentScene(delegate(Scene scene) { scene.HandleEditCommand(cmdparams); });
        }

        public bool TryGetScenePresence(UUID avatarId, out ScenePresence avatar)
        {
            lock (m_localScenes)
            {
                foreach (Scene scene in m_localScenes)
                {
                    if (scene.TryGetScenePresence(avatarId, out avatar))
                    {
                        return true;
                    }
                }
            }

            avatar = null;
            return false;
        }

        public bool TryGetRootScenePresence(UUID avatarId, out ScenePresence avatar)
        {
            lock (m_localScenes)
            {
                foreach (Scene scene in m_localScenes)
                {
                    avatar = scene.GetScenePresence(avatarId);

                    if (avatar != null && !avatar.IsChildAgent)
                        return true;
                }
            }

            avatar = null;
            return false;
        }

        public void CloseScene(Scene scene)
        {
            lock (m_localScenes)
                m_localScenes.Remove(scene);

            scene.Close();
        }

        public bool TryGetAvatarByName(string avatarName, out ScenePresence avatar)
        {
            lock (m_localScenes)
            {
                foreach (Scene scene in m_localScenes)
                {
                    if (scene.TryGetAvatarByName(avatarName, out avatar))
                    {
                        return true;
                    }
                }
            }

            avatar = null;
            return false;
        }

        public bool TryGetRootScenePresenceByName(string firstName, string lastName, out ScenePresence sp)
        {
            lock (m_localScenes)
            {
                foreach (Scene scene in m_localScenes)
                {
                    sp = scene.GetScenePresence(firstName, lastName);
                    if (sp != null && !sp.IsChildAgent)
                        return true;
                }
            }

            sp = null;
            return false;
        }

        public void ForEachScene(Action<Scene> action)
        {
            lock (m_localScenes)
                m_localScenes.ForEach(action);
        }
    }
}
