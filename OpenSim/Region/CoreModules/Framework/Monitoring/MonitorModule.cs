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

using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using log4net;
using Nini.Config;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.CoreModules.Framework.Monitoring.Alerts;
using OpenSim.Region.CoreModules.Framework.Monitoring.Monitors;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;

namespace OpenSim.Region.CoreModules.Framework.Monitoring
{
    public class MonitorModule : IRegionModule 
    {
        private Scene m_scene;
        private readonly List<IMonitor> m_monitors = new List<IMonitor>();
        private readonly List<IAlert> m_alerts = new List<IAlert>();
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public void DebugMonitors(string module, string[] args)
        {
            foreach (IMonitor monitor in m_monitors)
            {
                m_log.Info("[MonitorModule] " + m_scene.RegionInfo.RegionName + " reports " + monitor.GetName() + " = " + monitor.GetFriendlyValue());
            }
        }

        public void TestAlerts()
        {
            foreach (IAlert alert in m_alerts)
            {
                alert.Test();
            }
        }

        #region Implementation of IRegionModule

        public void Initialise(Scene scene, IConfigSource source)
        {
            m_scene = scene;

            m_scene.AddCommand(this, "monitor report",
                               "monitor report",
                               "Returns a variety of statistics about the current region and/or simulator",
                               DebugMonitors);

            MainServer.Instance.AddHTTPHandler("/monitorstats/" + m_scene.RegionInfo.RegionID, StatsPage);
        }

        public Hashtable StatsPage(Hashtable request)
        {
            // If request was for a specific monitor
            // eg url/?monitor=Monitor.Name
            if (request.ContainsKey("monitor"))
            {
                string monID = (string) request["monitor"];

                foreach (IMonitor monitor in m_monitors)
                {
                    string elemName = monitor.ToString();
                    if (elemName.StartsWith(monitor.GetType().Namespace))
                        elemName = elemName.Substring(monitor.GetType().Namespace.Length + 1);
                    if (elemName == monID || monitor.ToString() == monID)
                    {
                        Hashtable ereply3 = new Hashtable();

                        ereply3["int_response_code"] = 404; // 200 OK
                        ereply3["str_response_string"] = monitor.GetValue().ToString();
                        ereply3["content_type"] = "text/plain";

                        return ereply3;
                    }
                }

                // No monitor with that name
                Hashtable ereply2 = new Hashtable();

                ereply2["int_response_code"] = 404; // 200 OK
                ereply2["str_response_string"] = "No such monitor";
                ereply2["content_type"] = "text/plain";

                return ereply2;
            }

            string xml = "<data>";
            foreach (IMonitor monitor in m_monitors)
            {
                string elemName = monitor.ToString();
                if (elemName.StartsWith(monitor.GetType().Namespace))
                    elemName = elemName.Substring(monitor.GetType().Namespace.Length + 1);

                xml += "<" + elemName + ">" + monitor.GetValue() + "</" + elemName + ">";
            }
            xml += "</data>";

            Hashtable ereply = new Hashtable();

            ereply["int_response_code"] = 200; // 200 OK
            ereply["str_response_string"] = xml;
            ereply["content_type"] = "text/xml";

            return ereply;
        }

        public void PostInitialise()
        {
            m_monitors.Add(new AgentCountMonitor(m_scene));
            m_monitors.Add(new ChildAgentCountMonitor(m_scene));
            m_monitors.Add(new GCMemoryMonitor());
            m_monitors.Add(new ObjectCountMonitor(m_scene));
            m_monitors.Add(new PhysicsFrameMonitor(m_scene));
            m_monitors.Add(new PhysicsUpdateFrameMonitor(m_scene));
            m_monitors.Add(new PWSMemoryMonitor());
            m_monitors.Add(new ThreadCountMonitor());
            m_monitors.Add(new TotalFrameMonitor(m_scene));
            m_monitors.Add(new EventFrameMonitor(m_scene));
            m_monitors.Add(new LandFrameMonitor(m_scene));
            m_monitors.Add(new LastFrameTimeMonitor(m_scene));

            m_alerts.Add(new DeadlockAlert(m_monitors.Find(x => x is LastFrameTimeMonitor) as LastFrameTimeMonitor));

            foreach (IAlert alert in m_alerts)
            {
                alert.OnTriggerAlert += OnTriggerAlert;
            }
        }

        void OnTriggerAlert(System.Type reporter, string reason, bool fatal)
        {
            m_log.Error("[Monitor] " + reporter.Name + " for " + m_scene.RegionInfo.RegionName + " reports " + reason + " (Fatal: " + fatal + ")");
        }

        public void Close()
        {
            
        }

        public string Name
        {
            get { return "Region Health Monitoring Module"; }
        }

        public bool IsSharedModule
        {
            get { return false; }
        }

        #endregion
    }
}
