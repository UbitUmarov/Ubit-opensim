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

using System.Net;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Framework.Scenes;
using GridRegion = OpenSim.Services.Interfaces.GridRegion;

namespace OpenSim.Region.ClientStack.LindenUDP.Tests
{
    /// <summary>
    /// Mock scene for unit tests
    /// </summary>
    public class MockScene : SceneBase
    {
        public int ObjectNameCallsReceived
        {
            get { return m_objectNameCallsReceived; }
        }
        protected int m_objectNameCallsReceived;
        
        public MockScene()
        {
            m_regInfo = new RegionInfo(1000, 1000, null, null);
            m_regStatus = RegionStatus.Up;
        }
        
        public override void Update() {}
        public override void LoadWorldMap() {}
        
        public override ISceneAgent AddNewClient(IClientAPI client, PresenceType type)
        {
            client.OnObjectName += RecordObjectNameCall;

            // FIXME
            return null;
        }
        
        public override void RemoveClient(UUID agentID, bool someReason) {}
//        public override void CloseAllAgents(uint circuitcode) {}
        public override bool CheckClient(UUID clientId, IPEndPoint endPoint) { return true; }
        public override void OtherRegionUp(GridRegion otherRegion) {  }

        public override bool TryGetScenePresence(UUID uuid, out ScenePresence sp) { sp = null; return false; }
            
        /// <summary>
        /// Doesn't really matter what the call is - we're using this to test that a packet has actually been received
        /// </summary>
        protected void RecordObjectNameCall(IClientAPI remoteClient, uint localID, string message)
        {
            m_objectNameCallsReceived++;
        }
    }
}
