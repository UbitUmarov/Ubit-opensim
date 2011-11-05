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
using System.Net;
using System.IO;
using System.Text;

using log4net;
using Nini.Config;
using OpenMetaverse;
using OpenMetaverse.StructuredData;
using OpenSim.Services.Interfaces;

using OpenSim.Framework;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;

namespace OpenSim.Region.OptionalModules.Scripting.RegionReady
{
    public class RegionReadyModule : INonSharedRegionModule
    {
        private static readonly ILog m_log = 
            LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private IConfig m_config = null;
        private bool m_firstEmptyCompileQueue;
        private bool m_oarFileLoading;
        private bool m_lastOarLoadedOk;
        private int m_channelNotify = -1000;
        private bool m_enabled = false;
        private bool m_disable_logins = false;
        private string m_uri = string.Empty;
        
        Scene m_scene = null;
        
        #region INonSharedRegionModule interface

        public Type ReplaceableInterface 
        { 
            get { return null; }
        }
            
        public void Initialise(IConfigSource config)
        {
            //m_log.Info("[RegionReady] Initialising");

            m_config = config.Configs["RegionReady"];
            if (m_config != null) 
            {
                m_enabled = m_config.GetBoolean("enabled", false);
                
                if (m_enabled) 
                {
                    m_channelNotify = m_config.GetInt("channel_notify", m_channelNotify);
                    m_disable_logins = m_config.GetBoolean("login_disable", false);
                    m_uri = m_config.GetString("alert_uri",string.Empty);
                }
            }

//            if (!m_enabled)
//                m_log.Info("[RegionReady] disabled.");
        }

        public void AddRegion(Scene scene)
        {
            if (!m_enabled)
                return;

            m_firstEmptyCompileQueue = true;
            m_oarFileLoading = false;
            m_lastOarLoadedOk = true;

            m_scene = scene;

            m_scene.EventManager.OnEmptyScriptCompileQueue += OnEmptyScriptCompileQueue;
            m_scene.EventManager.OnOarFileLoaded += OnOarFileLoaded;
            m_scene.EventManager.OnLoginsEnabled += OnLoginsEnabled;

            m_log.DebugFormat("[RegionReady]: Enabled for region {0}", scene.RegionInfo.RegionName);

            if (m_disable_logins == true)
            {
                scene.LoginLock = true;
                scene.LoginsDisabled = true;
                m_log.InfoFormat("[RegionReady]: Logins disabled for {0}",m_scene.RegionInfo.RegionName);

                if(m_uri != string.Empty)
                {
                    RRAlert("disabled");
                }
            }
        }

        public void RemoveRegion(Scene scene)
        {
            if (!m_enabled)
                return;

            m_scene.EventManager.OnEmptyScriptCompileQueue -= OnEmptyScriptCompileQueue;
            m_scene.EventManager.OnOarFileLoaded -= OnOarFileLoaded;

            if(m_uri != string.Empty)
            {
                RRAlert("shutdown");
            }

            m_scene = null;
        }

        public void Close()
        {
        }

        public void RegionLoaded(Scene scene)
        {
        }

        public string Name
        {
            get { return "RegionReadyModule"; }
        }

        #endregion
        
        void OnEmptyScriptCompileQueue(int numScriptsFailed, string message)
        {
            if (m_firstEmptyCompileQueue || m_oarFileLoading) 
            {
                OSChatMessage c = new OSChatMessage();
                if (m_firstEmptyCompileQueue) 
                    c.Message = "server_startup,";
                else 
                    c.Message = "oar_file_load,";
                m_firstEmptyCompileQueue = false;
                m_oarFileLoading = false;

                m_scene.Backup(false);

                c.From = "RegionReady";
                if (m_lastOarLoadedOk) 
                    c.Message += "1,";
                else
                    c.Message += "0,";
                c.Channel = m_channelNotify;
                c.Message += numScriptsFailed.ToString() + "," + message;
                c.Type = ChatTypeEnum.Region;
                c.Position = new Vector3(((int)Constants.RegionSize * 0.5f), ((int)Constants.RegionSize * 0.5f), 30);
                c.Sender = null;
                c.SenderUUID = UUID.Zero;
                c.Scene = m_scene;

                m_log.InfoFormat("[RegionReady]: Region \"{0}\" is ready: \"{1}\" on channel {2}",
                                 m_scene.RegionInfo.RegionName, c.Message, m_channelNotify);

                m_scene.EventManager.TriggerOnChatBroadcast(this, c);
                m_scene.EventManager.TriggerLoginsEnabled(m_scene.RegionInfo.RegionName);
                m_scene.SceneGridService.InformNeighborsThatRegionisUp(m_scene.RequestModuleInterface<INeighbourService>(), m_scene.RegionInfo);
            }
        }

        void OnOarFileLoaded(Guid requestId, string message)
        {
            m_oarFileLoading = true;
            if (message==String.Empty) 
            {
                m_lastOarLoadedOk = true;
            } else {
                m_log.InfoFormat("[RegionReady]: Oar file load errors: {0}", message);
                m_lastOarLoadedOk = false;
            }
        }

        void OnLoginsEnabled(string regionName)
        {
            if (m_disable_logins == true)
            {
                if (m_scene.StartDisabled == false)
                {
                    m_scene.LoginsDisabled = false;
                    m_scene.LoginLock = false;
                    m_log.InfoFormat("[RegionReady]: Logins enabled for {0}", m_scene.RegionInfo.RegionName);
                    if ( m_uri != string.Empty )
                    {
                        RRAlert("enabled");
                    }
                }
            }
        }

        public void RRAlert(string status)
        {
            string request_method = "POST";
            string content_type = "application/json";
            OSDMap RRAlert = new OSDMap();

            RRAlert["alert"] = "region_ready";
            RRAlert["login"] = status;
            RRAlert["region_name"] = m_scene.RegionInfo.RegionName;
            RRAlert["region_id"] = m_scene.RegionInfo.RegionID;

            string strBuffer = "";
            byte[] buffer = new byte[1];
            try
            {
                strBuffer = OSDParser.SerializeJsonString(RRAlert);
                Encoding str = Util.UTF8;
                buffer = str.GetBytes(strBuffer);

            }
            catch (Exception e)
            {
                m_log.WarnFormat("[RegionReady]: Exception thrown on alert: {0}", e.Message);
            }

            WebRequest request = WebRequest.Create(m_uri);
            request.Method = request_method;
            request.ContentType = content_type;

            Stream os = null;
            try
            {
                request.ContentLength = buffer.Length;
                os = request.GetRequestStream();
                os.Write(buffer, 0, strBuffer.Length);
            }
            catch(Exception e)
            {
                m_log.WarnFormat("[RegionReady]: Exception thrown sending alert: {0}", e.Message);
            }
            finally
            {
                if (os != null)
                    os.Close();
            }
        }
    }
}
