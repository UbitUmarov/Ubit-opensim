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
//using System.Collections.Generic;
using System.Timers;
using OpenMetaverse.Packets;
using OpenSim.Framework;
using OpenSim.Framework.Statistics;
using OpenSim.Region.Framework.Interfaces;

namespace OpenSim.Region.Framework.Scenes
{
    public class SimStatsReporter
    {
//        private static readonly log4net.ILog m_log
//            = log4net.LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        public delegate void SendStatResult(SimStats stats);

        public delegate void YourStatsAreWrong();

        public event SendStatResult OnSendStatsResult;

        public event YourStatsAreWrong OnStatsIncorrect;

        private SendStatResult handlerSendStatResult = null;

        private YourStatsAreWrong handlerStatsIncorrect = null;

        public enum Stats : uint
        {
            TimeDilation = 0,
            SimFPS = 1,
            PhysicsFPS = 2,
            AgentUpdates = 3,
            FrameMS = 4,
            NetMS = 5,
            OtherMS = 6,
            PhysicsMS = 7,
            AgentMS = 8,
            ImageMS = 9,
            ScriptMS = 10,
            TotalPrim = 11,
            ActivePrim = 12,
            Agents = 13,
            ChildAgents = 14,
            ActiveScripts = 15,
            ScriptLinesPerSecond = 16,
            InPacketsPerSecond = 17,
            OutPacketsPerSecond = 18,
            PendingDownloads = 19,
            PendingUploads = 20,
            VirtualSizeKB = 21,
            ResidentSizeKB = 22,
            PendingLocalUploads = 23,
            UnAckedBytes = 24,
            PhysicsPinnedTasks = 25,
            PhysicsLODTasks = 26,
            PhysicsStepMS = 27,
            PhysicsShapeMS = 28,
            PhysicsOtherMS = 29,
            PhysicsMemory = 30,
            ScriptEPS = 31,
            SimSpareTime = 32,
            SimSleepTime = 33,
            IOPumpTime = 34
        }

        /// <summary>
        /// This is for llGetRegionFPS
        /// </summary>
        public float LastReportedSimFPS
        {
            get { return lastReportedSimFPS; }
        }

        /// <summary>
        /// Number of object updates performed in the last stats cycle
        /// </summary>
        /// <remarks>
        /// This isn't sent out to the client but it is very useful data to detect whether viewers are being sent a
        /// large number of object updates.
        /// </remarks>
        public float LastReportedObjectUpdates { get; private set; }

        public float[] LastReportedSimStats
        {
            get { return lastReportedSimStats; }
        }

        // Sending a stats update every 3 seconds-
        private int statsUpdatesEveryMS = 3000;
        private float statsUpdateFactor = 0;
        private float m_timeDilation = 0;
        private int m_fps = 0;

        // saved last reported value so there is something available for llGetRegionFPS 
        private float lastReportedSimFPS = 0;
        private float[] lastReportedSimStats = new float[23];
        private float m_pfps = 0;

        /// <summary>
        /// Number of agent updates requested in this stats cycle
        /// </summary>
        private int m_agentUpdates = 0;

        private float m_frameMS = 0;
        private int m_objectUpdates;
        private int m_netMS = 0;
        private int m_agentMS = 0;
        private float m_physicsMS = 0;
        private int m_imageMS = 0;
        private float m_otherMS = 0;
        private float m_sleeptimeMS = 0;

//Ckrinke: (3-21-08) Comment out to remove a compiler warning. Bring back into play when needed.
//Ckrinke        private int m_scriptMS = 0;

        private int m_rootAgents = 0;
        private int m_childAgents = 0;
        private int m_numPrim = 0;
        private int m_inPacketsPerSecond = 0;
        private int m_outPacketsPerSecond = 0;
        private int m_activePrim = 0;
        private int m_unAckedBytes = 0;
        private int m_pendingDownloads = 0;
        private int m_pendingUploads = 0;
        private int m_activeScripts = 0;
        private int m_scriptLinesPerSecond = 0;

        private int m_objectCapacity = 45000;

        private Scene m_scene;

        private RegionInfo ReportingRegion;

        private Timer m_report = new Timer();

        private IEstateModule estateModule;

        public SimStatsReporter(Scene scene)
        {

            m_scene = scene;
            statsUpdateFactor = (float)(statsUpdatesEveryMS / 1000);
            ReportingRegion = scene.RegionInfo;

            m_objectCapacity = scene.RegionInfo.ObjectCapacity;
            m_report.AutoReset = true;
            m_report.Interval = statsUpdatesEveryMS;
            m_report.Elapsed += new ElapsedEventHandler(statsHeartBeat);
            m_report.Enabled = true;

            if (StatsManager.SimExtraStats != null)
                OnSendStatsResult += StatsManager.SimExtraStats.ReceiveClassicSimStatsPacket;
        }

        public void SetUpdateMS(int ms)
        {
            statsUpdatesEveryMS = ms;
            statsUpdateFactor = (float)(statsUpdatesEveryMS / 1000);
            m_report.Interval = statsUpdatesEveryMS;
        }

        private void statsHeartBeat(object sender, EventArgs e)
        {
            SimStatsPacket.StatBlock[] sb = new SimStatsPacket.StatBlock[23];
            SimStatsPacket.RegionBlock rb = new SimStatsPacket.RegionBlock();
            
            float factorByframe;
            float factor;

            // Know what's not thread safe in Mono... modifying timers.
            // m_log.Debug("Firing Stats Heart Beat");
            lock (m_report)
            {
                uint regionFlags = 0;
                
                try
                {
                    if (estateModule == null)
                        estateModule = m_scene.RequestModuleInterface<IEstateModule>();
                    regionFlags = estateModule != null ? estateModule.GetRegionFlags() : (uint) 0;
                }
                catch (Exception)
                {
                    // leave region flags at 0
                }

#region various statistic googly moogly  

                // Our FPS is actually 10fps, so multiplying by 5 to get the amount that people expect there
                // 0-50 is pretty close to 0-45
 // show real
                float simfps = (int) ((m_fps));
                if (simfps == 0.0)
                    simfps = 10; //  we still don't have stats on this

                // factor to convert things to per second
                factor = 1 / statsUpdateFactor;
                // factor to convert things for time per frame need because how acumulators work
                //                factorByframe = factor / simfps;
                factorByframe = 1 / simfps;
                				
                // save the reported value so there is something available for llGetRegionFPS 
                lastReportedSimFPS = simfps * factor;

                float physfps = ((m_pfps));

                if (physfps < 0)
                    physfps = 0;

#endregion
                
                for (int i = 0; i < 23; i++)
                {
                    sb[i] = new SimStatsPacket.StatBlock();
                }
                
                sb[0].StatID = (uint) Stats.TimeDilation;
                sb[0].StatValue = (Single.IsNaN(m_timeDilation)) ? 0.1f : m_timeDilation ; //((((m_timeDilation + (0.10f * statsUpdateFactor)) /10)  / statsUpdateFactor));

                sb[1].StatID = (uint) Stats.SimFPS;
                sb[1].StatValue = simfps * factor;
				
                sb[2].StatID = (uint) Stats.PhysicsFPS;
                sb[2].StatValue = physfps * factor;

                sb[3].StatID = (uint) Stats.AgentUpdates;
                sb[3].StatValue = (m_agentUpdates * factor);

                sb[4].StatID = (uint) Stats.Agents;
                sb[4].StatValue = m_rootAgents;

                sb[5].StatID = (uint) Stats.ChildAgents;
                sb[5].StatValue = m_childAgents;

                sb[6].StatID = (uint) Stats.TotalPrim;
                sb[6].StatValue = m_numPrim;

                sb[7].StatID = (uint) Stats.ActivePrim;
                sb[7].StatValue = m_activePrim;

                sb[8].StatID = (uint)Stats.FrameMS;
                //                sb[8].StatValue = m_frameMS * factorByframe;
                float simFrameTime = 1000.0f / (simfps * factor);
                sb[8].StatValue = simFrameTime;

                sb[9].StatID = (uint)Stats.NetMS;
                sb[9].StatValue = m_netMS * factorByframe;

                sb[10].StatID = (uint)Stats.PhysicsMS;
                sb[10].StatValue = m_physicsMS * factorByframe;

                sb[11].StatID = (uint)Stats.ImageMS ;
                sb[11].StatValue = m_imageMS * factorByframe;

                sb[12].StatID = (uint)Stats.OtherMS;
                float othertmp = m_frameMS - m_physicsMS - m_imageMS - m_netMS - m_agentMS;
                if (othertmp < 0)
                    othertmp = 0;

                sb[12].StatValue = othertmp * factorByframe;

                sb[13].StatID = (uint)Stats.InPacketsPerSecond;
                sb[13].StatValue = (m_inPacketsPerSecond * factor);

                sb[14].StatID = (uint)Stats.OutPacketsPerSecond;
                sb[14].StatValue = (m_outPacketsPerSecond * factor);

                sb[15].StatID = (uint)Stats.UnAckedBytes;
                sb[15].StatValue = m_unAckedBytes;

                sb[16].StatID = (uint)Stats.AgentMS;
                sb[16].StatValue = m_agentMS * factorByframe;

                sb[17].StatID = (uint)Stats.PendingDownloads;
                sb[17].StatValue = m_pendingDownloads;

                sb[18].StatID = (uint)Stats.PendingUploads;
                sb[18].StatValue = m_pendingUploads;

                sb[19].StatID = (uint)Stats.ActiveScripts;
                sb[19].StatValue = m_activeScripts;

                sb[20].StatID = (uint)Stats.ScriptLinesPerSecond;
                sb[20].StatValue = m_scriptLinesPerSecond * factor;

                float spare = m_scene.m_simframetime - m_frameMS * factorByframe;
                if (spare < 0)
                    spare = 0;
                sb[21].StatID = (uint)Stats.SimSpareTime;
                sb[21].StatValue = spare;

                sb[22].StatID = (uint)Stats.SimSleepTime;
                sb[22].StatValue = m_sleeptimeMS * factorByframe;

                for (int i = 0; i < 23; i++)
                {
                    lastReportedSimStats[i] = sb[i].StatValue;
                }
              
                SimStats simStats 
                    = new SimStats(
                        ReportingRegion.RegionLocX, ReportingRegion.RegionLocY, regionFlags, (uint)m_objectCapacity,
                        rb, sb, m_scene.RegionInfo.originRegionID);

                handlerSendStatResult = OnSendStatsResult;
                if (handlerSendStatResult != null)
                {
                    handlerSendStatResult(simStats);
                }

                // Extra statistics that aren't currently sent to clients
                LastReportedObjectUpdates = m_objectUpdates / statsUpdateFactor;

                resetvalues();
            }
        }

        private void resetvalues()
        {
            m_timeDilation = 0;
            m_fps = 0;
            m_pfps = 0;
            m_agentUpdates = 0;
            m_objectUpdates = 0;
            //m_inPacketsPerSecond = 0;
            //m_outPacketsPerSecond = 0;
            m_unAckedBytes = 0;
            m_scriptLinesPerSecond = 0;

            m_frameMS = 0;
            m_agentMS = 0;
            m_netMS = 0;
            m_physicsMS = 0;
            m_imageMS = 0;
            m_otherMS = 0;
            m_sleeptimeMS = 0;

//Ckrinke This variable is not used, so comment to remove compiler warning until it is used.
//Ckrinke            m_scriptMS = 0;
        }

        # region methods called from Scene
        // The majority of these functions are additive
        // so that you can easily change the amount of
        // seconds in between sim stats updates

        public void AddTimeDilation(float td)
        {
            //float tdsetting = td;
            //if (tdsetting > 1.0f)
                //tdsetting = (tdsetting - (tdsetting - 0.91f));

            //if (tdsetting < 0)
                //tdsetting = 0.0f;
            m_timeDilation = td;
        }

        public void SetRootAgents(int rootAgents)
        {
            m_rootAgents = rootAgents;
            CheckStatSanity();

        }

        internal void CheckStatSanity()
        {
            if (m_rootAgents < 0 || m_childAgents < 0)
            {
                handlerStatsIncorrect = OnStatsIncorrect;
                if (handlerStatsIncorrect != null)
                {
                    handlerStatsIncorrect();
                }
            }
            if (m_rootAgents == 0 && m_childAgents == 0)
            {
                m_unAckedBytes = 0;
            }
        }

        public void SetChildAgents(int childAgents)
        {
            m_childAgents = childAgents;
            CheckStatSanity();
        }

        public void SetObjects(int objects)
        {
            m_numPrim = objects;
        }

        public void SetActiveObjects(int objects)
        {
            m_activePrim = objects;
        }

        public void AddFPS(int frames)
        {
            m_fps += frames;
        }

        public void AddPhysicsFPS(float frames)
        {
            m_pfps += frames;
        }

        public void AddObjectUpdates(int numUpdates)
        {
            m_objectUpdates += numUpdates;
        }

        public void AddAgentUpdates(int numUpdates)
        {
            m_agentUpdates += numUpdates;
        }

        public void AddInPackets(int numPackets)
        {
            m_inPacketsPerSecond = numPackets;
        }

        public void AddOutPackets(int numPackets)
        {
            m_outPacketsPerSecond = numPackets;
        }

        public void AddunAckedBytes(int numBytes)
        {
            m_unAckedBytes += numBytes;
            if (m_unAckedBytes < 0) m_unAckedBytes = 0;
        }

        public void addFrameMS(float ms)
        {
            m_frameMS += ms;
        }

        public void addNetMS(int ms)
        {
            m_netMS += ms;
        }

        public void addAgentMS(int ms)
        {
            m_agentMS += ms;
        }
        public void addPhysicsMS(float ms)
        {
            m_physicsMS += ms;
        }

        public void addImageMS(int ms)
        {
            m_imageMS += ms;
        }
        public void addOtherMS(float ms)
        {
            m_otherMS += ms;
        }

        public void addSleepMS(float ms)
        {
            m_sleeptimeMS += ms;
        }

        public void AddPendingDownloads(int count)
        {
            m_pendingDownloads += count;
            if (m_pendingDownloads < 0) m_pendingDownloads = 0;
            //m_log.InfoFormat("[stats]: Adding {0} to pending downloads to make {1}", count, m_pendingDownloads);
        }

        public void addScriptLines(int count)
        {
            m_scriptLinesPerSecond += count;
        }

        public void SetActiveScripts(int count)
        {
            m_activeScripts = count;
        }

        public void AddPacketsStats(int inPackets, int outPackets, int unAckedBytes)
        {
            AddInPackets(inPackets);
            AddOutPackets(outPackets);
            AddunAckedBytes(unAckedBytes);
        }

        #endregion
    }
}
