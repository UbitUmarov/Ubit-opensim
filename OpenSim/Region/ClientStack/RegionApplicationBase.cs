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

using System.Collections.Generic;
using System.Net;
using System.Reflection;
using log4net;
using Nini.Config;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Framework.Communications;
using OpenSim.Framework.Servers;
using OpenSim.Framework.Servers.HttpServer;
using OpenSim.Region.Framework;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;
using OpenSim.Region.Physics.Manager;

namespace OpenSim.Region.ClientStack
{
    public abstract class RegionApplicationBase : BaseOpenSimServer
    {
        private static readonly ILog m_log
            = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        protected Dictionary<EndPoint, uint> m_clientCircuits = new Dictionary<EndPoint, uint>();
        protected NetworkServersInfo m_networkServersInfo;
        protected uint m_httpServerPort;
        protected ISimulationDataService m_simulationDataService;
        protected IEstateDataService m_estateDataService;
        protected ClientStackManager m_clientStackManager;
        protected SceneManager m_sceneManager = new SceneManager();

        public SceneManager SceneManager { get { return m_sceneManager; } }
        public NetworkServersInfo NetServersInfo { get { return m_networkServersInfo; } }
        public ISimulationDataService SimulationDataService { get { return m_simulationDataService; } }
        public IEstateDataService EstateDataService { get { return m_estateDataService; } }
       
        protected abstract void Initialize();
        
        /// <summary>
        /// Get a new physics scene.
        /// </summary>
        /// 
        /// <param name="osSceneIdentifier">
        /// The region of the OpenSim scene this physics scene is serving.  This will be used in log messages.
        /// </param>
        /// <returns></returns>
        protected abstract PhysicsScene GetPhysicsScene(RegionInfo region);
        
        protected abstract ClientStackManager CreateClientStackManager();
        protected abstract Scene CreateScene(RegionInfo regionInfo, ISimulationDataService simDataService, IEstateDataService estateDataService, AgentCircuitManager circuitManager);

        protected override void StartupSpecific()
        {
            m_clientStackManager = CreateClientStackManager();

            Initialize();

            m_httpServer 
                = new BaseHttpServer(
                    m_httpServerPort, m_networkServersInfo.HttpUsesSSL, m_networkServersInfo.httpSSLPort, 
                    m_networkServersInfo.HttpSSLCN);
            
            if (m_networkServersInfo.HttpUsesSSL && (m_networkServersInfo.HttpListenerPort == m_networkServersInfo.httpSSLPort))
            {
                m_log.Error("[REGION SERVER]: HTTP Server config failed.   HTTP Server and HTTPS server must be on different ports");
            }

            m_log.InfoFormat("[REGION SERVER]: Starting HTTP server on port {0}", m_httpServerPort);
            m_httpServer.Start();

            MainServer.Instance = m_httpServer;

            // "OOB" Server
            if (m_networkServersInfo.ssl_listener)
            {
                BaseHttpServer server = null;
                server = new BaseHttpServer(
                    m_networkServersInfo.https_port, m_networkServersInfo.ssl_listener, m_networkServersInfo.cert_path,
                    m_networkServersInfo.cert_pass);
                // Add the server to m_Servers
                if(server != null)
                {
                    m_log.InfoFormat("[REGION SERVER]: Starting HTTPS server on port {0}", server.Port);
                    MainServer.AddHttpServer(server);
                    server.Start();
                }
            }

            base.StartupSpecific();
        }

        /// <summary>
        /// Get a new physics scene.
        /// </summary>
        /// <param name="engine">The name of the physics engine to use</param>
        /// <param name="meshEngine">The name of the mesh engine to use</param>
        /// <param name="config">The configuration data to pass to the physics and mesh engines</param>
        /// <param name="osSceneIdentifier">
        /// The name of the OpenSim scene this physics scene is serving.  This will be used in log messages.
        /// </param>
        /// <returns></returns>
        protected PhysicsScene GetPhysicsScene(
            string engine, string meshEngine, IConfigSource config, RegionInfo region)
        {
            PhysicsPluginManager physicsPluginManager;
            physicsPluginManager = new PhysicsPluginManager();
            physicsPluginManager.LoadPluginsFromAssemblies("Physics");

            return physicsPluginManager.GetPhysicsScene(engine, meshEngine, config, region);
        }
    }
}