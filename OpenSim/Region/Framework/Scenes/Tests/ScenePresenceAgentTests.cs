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
using System.Text;
using System.Threading;
using System.Timers;
using Timer=System.Timers.Timer;
using Nini.Config;
using NUnit.Framework;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Framework.Communications;
using OpenSim.Region.Framework.Scenes;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.CoreModules.Framework.EntityTransfer;
using OpenSim.Region.CoreModules.World.Serialiser;
using OpenSim.Region.CoreModules.ServiceConnectorsOut.Simulation;
using OpenSim.Tests.Common;
using OpenSim.Tests.Common.Mock;

namespace OpenSim.Region.Framework.Scenes.Tests
{
    /// <summary>
    /// Scene presence tests
    /// </summary>
    [TestFixture]
    public class ScenePresenceAgentTests
    {
        public Scene scene, scene2, scene3;
        public UUID agent1, agent2, agent3;
        public static Random random;
        public ulong region1,region2,region3;
        public AgentCircuitData acd1;
        public SceneObjectGroup sog1, sog2, sog3;
        public TestClient testclient;

        [TestFixtureSetUp]
        public void Init()
        {
            TestHelpers.InMethod();
            
            scene = SceneHelpers.SetupScene("Neighbour x", UUID.Random(), 1000, 1000);
            scene2 = SceneHelpers.SetupScene("Neighbour x+1", UUID.Random(), 1001, 1000);
            scene3 = SceneHelpers.SetupScene("Neighbour x-1", UUID.Random(), 999, 1000);

            ISharedRegionModule interregionComms = new LocalSimulationConnectorModule();
            interregionComms.Initialise(new IniConfigSource());
            interregionComms.PostInitialise();
            SceneHelpers.SetupSceneModules(scene, new IniConfigSource(), interregionComms);
            SceneHelpers.SetupSceneModules(scene2, new IniConfigSource(), interregionComms);
            SceneHelpers.SetupSceneModules(scene3, new IniConfigSource(), interregionComms);

            agent1 = UUID.Random();
            agent2 = UUID.Random();
            agent3 = UUID.Random();
            random = new Random();
            sog1 = SceneHelpers.CreateSceneObject(1, agent1);
            scene.AddSceneObject(sog1);
            sog2 = SceneHelpers.CreateSceneObject(1, agent1);
            scene.AddSceneObject(sog2);
            sog3 = SceneHelpers.CreateSceneObject(1, agent1);
            scene.AddSceneObject(sog3);

            region1 = scene.RegionInfo.RegionHandle;
            region2 = scene2.RegionInfo.RegionHandle;
            region3 = scene3.RegionInfo.RegionHandle;
        }

        [Test]
        public void TestCloseAgent()
        {
            TestHelpers.InMethod();
//            log4net.Config.XmlConfigurator.Configure();

            TestScene scene = SceneHelpers.SetupScene();
            ScenePresence sp = SceneHelpers.AddScenePresence(scene, TestHelpers.ParseTail(0x1));

            Assert.That(scene.AuthenticateHandler.GetAgentCircuitData(sp.UUID), Is.Not.Null);

            scene.IncomingCloseAgent(sp.UUID);

            Assert.That(scene.GetScenePresence(sp.UUID), Is.Null);
            Assert.That(scene.AuthenticateHandler.GetAgentCircuitData(sp.UUID), Is.Null);
        }

        /// <summary>
        /// Test that if a root agent logs into a region, a child agent is also established in the neighbouring region
        /// </summary>
        /// <remarks>
        /// Please note that unlike the other tests here, this doesn't rely on structures
        /// </remarks>
        [Test]
        public void TestChildAgentEstablished()
        {
            TestHelpers.InMethod();
//            log4net.Config.XmlConfigurator.Configure();
            
            UUID agent1Id = UUID.Parse("00000000-0000-0000-0000-000000000001");
            
            TestScene myScene1 = SceneHelpers.SetupScene("Neighbour y", UUID.Random(), 1000, 1000);
//            TestScene myScene2 = SceneHelpers.SetupScene("Neighbour y + 1", UUID.Random(), 1001, 1000);
            
            IConfigSource configSource = new IniConfigSource();
            configSource.AddConfig("Modules").Set("EntityTransferModule", "BasicEntityTransferModule");                      
            EntityTransferModule etm = new EntityTransferModule();
            
            SceneHelpers.SetupSceneModules(myScene1, configSource, etm);            
            
            SceneHelpers.AddScenePresence(myScene1, agent1Id);
//            ScenePresence childPresence = myScene2.GetScenePresence(agent1);

            // TODO: Need to do a fair amount of work to allow synchronous establishment of child agents
//            Assert.That(childPresence, Is.Not.Null);
//            Assert.That(childPresence.IsChildAgent, Is.True);
        }

//        /// <summary>
//        /// Test adding a root agent to a scene.  Doesn't yet actually complete crossing the agent into the scene.
//        /// </summary>
//        [Test]
//        public void T010_TestAddRootAgent()
//        {
//            TestHelpers.InMethod();
//
//            string firstName = "testfirstname";
//
//            AgentCircuitData agent = new AgentCircuitData();
//            agent.AgentID = agent1;
//            agent.firstname = firstName;
//            agent.lastname = "testlastname";
//            agent.SessionID = UUID.Random();
//            agent.SecureSessionID = UUID.Random();
//            agent.circuitcode = 123;
//            agent.BaseFolder = UUID.Zero;
//            agent.InventoryFolder = UUID.Zero;
//            agent.startpos = Vector3.Zero;
//            agent.CapsPath = GetRandomCapsObjectPath();
//            agent.ChildrenCapSeeds = new Dictionary<ulong, string>();
//            agent.child = true;
//
//            scene.PresenceService.LoginAgent(agent.AgentID.ToString(), agent.SessionID, agent.SecureSessionID);
//
//            string reason;
//            scene.NewUserConnection(agent, (uint)TeleportFlags.ViaLogin, out reason);
//            testclient = new TestClient(agent, scene);
//            scene.AddNewClient(testclient);
//
//            ScenePresence presence = scene.GetScenePresence(agent1);
//
//            Assert.That(presence, Is.Not.Null, "presence is null");
//            Assert.That(presence.Firstname, Is.EqualTo(firstName), "First name not same");
//            acd1 = agent;
//        }
//
//        /// <summary>
//        /// Test removing an uncrossed root agent from a scene.
//        /// </summary>
//        [Test]
//        public void T011_TestRemoveRootAgent()
//        {
//            TestHelpers.InMethod();
//
//            scene.RemoveClient(agent1);
//
//            ScenePresence presence = scene.GetScenePresence(agent1);
//
//            Assert.That(presence, Is.Null, "presence is not null");
//        }

        [Test]
        public void T012_TestAddNeighbourRegion()
        {
            TestHelpers.InMethod();

            string reason;

            if (acd1 == null)
                fixNullPresence();

            scene.NewUserConnection(acd1, 0, out reason);
            if (testclient == null)
                testclient = new TestClient(acd1, scene);
            scene.AddNewClient(testclient, PresenceType.User);

            ScenePresence presence = scene.GetScenePresence(agent1);
            presence.MakeRootAgent(new Vector3(90,90,90),false);

            string cap = presence.ControllingClient.RequestClientInfo().CapsPath;

            presence.AddNeighbourRegion(region2, cap);
            presence.AddNeighbourRegion(region3, cap);

            Assert.That(presence.KnownRegionCount, Is.EqualTo(2));
        }

        [Test]
        public void T013_TestRemoveNeighbourRegion()
        {
            TestHelpers.InMethod();

            ScenePresence presence = scene.GetScenePresence(agent1);
            presence.RemoveNeighbourRegion(region3);

            Assert.That(presence.KnownRegionCount,Is.EqualTo(1));
            /*
            presence.MakeChildAgent;
            presence.MakeRootAgent;
            CompleteAvatarMovement
            */
        }

        // I'm commenting this test because it does not represent
        // crossings. The Thread.Sleep's in here are not meaningful mocks,
        // and they sometimes fail in panda.
        // We need to talk in order to develop a test
        // that really tests region crossings. There are 3 async components,
        // but things are synchronous among them. So there should be
        // 3 threads in here.
        //[Test]
        public void T021_TestCrossToNewRegion()
        {
            TestHelpers.InMethod();

            scene.RegisterRegionWithGrid();
            scene2.RegisterRegionWithGrid();

            // Adding child agent to region 1001
            string reason;
            scene2.NewUserConnection(acd1,0, out reason);
            scene2.AddNewClient(testclient, PresenceType.User);

            ScenePresence presence = scene.GetScenePresence(agent1);
            presence.MakeRootAgent(new Vector3(0,unchecked(Constants.RegionSize-1),0), true);

            ScenePresence presence2 = scene2.GetScenePresence(agent1);

           // Adding neighbour region caps info to presence2

            string cap = presence.ControllingClient.RequestClientInfo().CapsPath;
            presence2.AddNeighbourRegion(region1, cap);

            Assert.That(presence.IsChildAgent, Is.False, "Did not start root in origin region.");
            Assert.That(presence2.IsChildAgent, Is.True, "Is not a child on destination region.");

            // Cross to x+1
            presence.AbsolutePosition = new Vector3(Constants.RegionSize+1,3,100);
            presence.Update();

            EventWaitHandle wh = new EventWaitHandle (false, EventResetMode.AutoReset, "Crossing");

            // Mimicking communication between client and server, by waiting OK from client
            // sent by TestClient.CrossRegion call. Originally, this is network comm.
            if (!wh.WaitOne(5000,false))
            {
                presence.Update();
                if (!wh.WaitOne(8000,false))
                    throw new ArgumentException("1 - Timeout waiting for signal/variable.");
            }

            // This is a TestClient specific method that fires OnCompleteMovementToRegion event, which
            // would normally be fired after receiving the reply packet from comm. done on the last line.
            testclient.CompleteMovement();

            // Crossings are asynchronous
            int timer = 10;

            // Make sure cross hasn't already finished
            if (!presence.IsInTransit && !presence.IsChildAgent)
            {
                // If not and not in transit yet, give it some more time
                Thread.Sleep(5000);
            }

            // Enough time, should at least be in transit by now.
            while (presence.IsInTransit && timer > 0)
            {
                Thread.Sleep(1000);
                timer-=1;
            }

            Assert.That(timer,Is.GreaterThan(0),"Timed out waiting to cross 2->1.");
            Assert.That(presence.IsChildAgent, Is.True, "Did not complete region cross as expected.");
            Assert.That(presence2.IsChildAgent, Is.False, "Did not receive root status after receiving agent.");

            // Cross Back
            presence2.AbsolutePosition = new Vector3(-10, 3, 100);
            presence2.Update();

            if (!wh.WaitOne(5000,false))
            {
                presence2.Update();
                if (!wh.WaitOne(8000,false))
                    throw new ArgumentException("2 - Timeout waiting for signal/variable.");
            }
            testclient.CompleteMovement();

            if (!presence2.IsInTransit && !presence2.IsChildAgent)
            {
                // If not and not in transit yet, give it some more time
                Thread.Sleep(5000);
            }

            // Enough time, should at least be in transit by now.
            while (presence2.IsInTransit && timer > 0)
            {
                Thread.Sleep(1000);
                timer-=1;
            }

            Assert.That(timer,Is.GreaterThan(0),"Timed out waiting to cross 1->2.");
            Assert.That(presence2.IsChildAgent, Is.True, "Did not return from region as expected.");
            Assert.That(presence.IsChildAgent, Is.False, "Presence was not made root in old region again.");
        }
        
        public void fixNullPresence()
        {
            string firstName = "testfirstname";

            AgentCircuitData agent = new AgentCircuitData();
            agent.AgentID = agent1;
            agent.firstname = firstName;
            agent.lastname = "testlastname";
            agent.SessionID = UUID.Zero;
            agent.SecureSessionID = UUID.Zero;
            agent.circuitcode = 123;
            agent.BaseFolder = UUID.Zero;
            agent.InventoryFolder = UUID.Zero;
            agent.startpos = Vector3.Zero;
            agent.CapsPath = GetRandomCapsObjectPath();
            agent.Appearance = new AvatarAppearance();

            acd1 = agent;
        }
        
        public static string GetRandomCapsObjectPath()
        {
            UUID caps = UUID.Random();
            string capsPath = caps.ToString();
            capsPath = capsPath.Remove(capsPath.Length - 4, 4);
            return capsPath;
        }
    }
}