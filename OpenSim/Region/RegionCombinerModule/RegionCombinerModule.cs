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
using log4net;
using Nini.Config;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Framework.Client;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;
using OpenSim.Framework.Console;
using OpenSim.Region.Physics.Manager;
using Mono.Addins;

[assembly: Addin("RegionCombinerModule", "0.1")]
[assembly: AddinDependency("OpenSim", "0.5")]
namespace OpenSim.Region.RegionCombinerModule
{

    [Extension(Path = "/OpenSim/RegionModules", NodeName = "RegionModule")]
    public class RegionCombinerModule : ISharedRegionModule
    {
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public string Name
        {
            get { return "RegionCombinerModule"; }
        }

        public Type ReplaceableInterface
        {
            get { return null; }
        }

        private Dictionary<UUID, RegionConnections> m_regions = new Dictionary<UUID, RegionConnections>();
        private bool enabledYN = false;
        private Dictionary<UUID, Scene> m_startingScenes = new Dictionary<UUID, Scene>();

        public void Initialise(IConfigSource source)
        {
            IConfig myConfig = source.Configs["Startup"];
            enabledYN = myConfig.GetBoolean("CombineContiguousRegions", false);
            //enabledYN = true;
            if (enabledYN)
                MainConsole.Instance.Commands.AddCommand("RegionCombinerModule", false, "fix-phantoms",
                    "Fix phantom objects", "Fixes phantom objects after an import to megaregions", FixPhantoms);
            }

        public void Close()
        {
        }

        public void AddRegion(Scene scene)
        {
        }

        public void RemoveRegion(Scene scene)
        {
        }

        public void RegionLoaded(Scene scene)
        {
            if (enabledYN)
            {
                RegionLoadedDoWork(scene);

                scene.EventManager.OnNewPresence += NewPresence;
            }
        }

        private void NewPresence(ScenePresence presence)
        {
            if (presence.IsChildAgent)
            {
                byte[] throttleData;

                try
                {
                    throttleData = presence.ControllingClient.GetThrottlesPacked(1);
                } 
                catch (NotImplementedException)
                {
                    return;
                }

                if (throttleData == null)
                    return;

                if (throttleData.Length == 0)
                    return;

                if (throttleData.Length != 28)
                    return;

                byte[] adjData;
                int pos = 0;

                if (!BitConverter.IsLittleEndian)
                {
                    byte[] newData = new byte[7 * 4];
                    Buffer.BlockCopy(throttleData, 0, newData, 0, 7 * 4);

                    for (int i = 0; i < 7; i++)
                        Array.Reverse(newData, i * 4, 4);

                    adjData = newData;
                }
                else
                {
                    adjData = throttleData;
                }

                // 0.125f converts from bits to bytes
                int resend = (int)(BitConverter.ToSingle(adjData, pos) * 0.125f); pos += 4;
                int land = (int)(BitConverter.ToSingle(adjData, pos) * 0.125f); pos += 4;
                int wind = (int)(BitConverter.ToSingle(adjData, pos) * 0.125f); pos += 4;
                int cloud = (int)(BitConverter.ToSingle(adjData, pos) * 0.125f); pos += 4;
                int task = (int)(BitConverter.ToSingle(adjData, pos) * 0.125f); pos += 4;
                int texture = (int)(BitConverter.ToSingle(adjData, pos) * 0.125f); pos += 4;
                int asset = (int)(BitConverter.ToSingle(adjData, pos) * 0.125f);
                // State is a subcategory of task that we allocate a percentage to


                //int total = resend + land + wind + cloud + task + texture + asset;

                byte[] data = new byte[7 * 4];
                int ii = 0;

                Buffer.BlockCopy(Utils.FloatToBytes(resend), 0, data, ii, 4); ii += 4;
                Buffer.BlockCopy(Utils.FloatToBytes(land * 50), 0, data, ii, 4); ii += 4;
                Buffer.BlockCopy(Utils.FloatToBytes(wind), 0, data, ii, 4); ii += 4;
                Buffer.BlockCopy(Utils.FloatToBytes(cloud), 0, data, ii, 4); ii += 4;
                Buffer.BlockCopy(Utils.FloatToBytes(task), 0, data, ii, 4); ii += 4;
                Buffer.BlockCopy(Utils.FloatToBytes(texture), 0, data, ii, 4); ii += 4;
                Buffer.BlockCopy(Utils.FloatToBytes(asset), 0, data, ii, 4);

                try
                {
                    presence.ControllingClient.SetChildAgentThrottle(data);
                }
                catch (NotImplementedException)
                {
                    return;
                }

            }
        }

        private void RegionLoadedDoWork(Scene scene)
        {
/* 
            // For testing on a single instance
            if (scene.RegionInfo.RegionLocX == 1004 && scene.RegionInfo.RegionLocY == 1000)
                return;
            // 
*/
            lock (m_startingScenes)
                m_startingScenes.Add(scene.RegionInfo.originRegionID, scene);

            // Give each region a standard set of non-infinite borders
            Border northBorder = new Border();
            northBorder.BorderLine = new Vector3(0, (int)Constants.RegionSize, (int)Constants.RegionSize);  //<---
            northBorder.CrossDirection = Cardinals.N;
            scene.NorthBorders[0] = northBorder;

            Border southBorder = new Border();
            southBorder.BorderLine = new Vector3(0, (int)Constants.RegionSize, 0);    //--->
            southBorder.CrossDirection = Cardinals.S;
            scene.SouthBorders[0] = southBorder;

            Border eastBorder = new Border();
            eastBorder.BorderLine = new Vector3(0, (int)Constants.RegionSize, (int)Constants.RegionSize);   //<---
            eastBorder.CrossDirection = Cardinals.E;
            scene.EastBorders[0] = eastBorder;

            Border westBorder = new Border();
            westBorder.BorderLine = new Vector3(0, (int)Constants.RegionSize, 0);     //--->
            westBorder.CrossDirection = Cardinals.W;
            scene.WestBorders[0] = westBorder;



            RegionConnections regionConnections = new RegionConnections();
            regionConnections.ConnectedRegions = new List<RegionData>();
            regionConnections.RegionScene = scene;
            regionConnections.RegionLandChannel = scene.LandChannel;
            regionConnections.RegionId = scene.RegionInfo.originRegionID;
            regionConnections.X = scene.RegionInfo.RegionLocX;
            regionConnections.Y = scene.RegionInfo.RegionLocY;
            regionConnections.XEnd = (int)Constants.RegionSize;
            regionConnections.YEnd = (int)Constants.RegionSize;


            lock (m_regions)
            {
                bool connectedYN = false;

                foreach (RegionConnections conn in m_regions.Values)
                {
                    #region commented
                    /*
                    // If we're one region over +x +y
                    //xxy
                    //xxx
                    //xxx
                    if ((((int)conn.X * (int)Constants.RegionSize) + conn.XEnd 
                        == (regionConnections.X * (int)Constants.RegionSize)) 
                        && (((int)conn.Y * (int)Constants.RegionSize) - conn.YEnd 
                        == (regionConnections.Y * (int)Constants.RegionSize)))
                    {
                        Vector3 offset = Vector3.Zero;
                        offset.X = (((regionConnections.X * (int) Constants.RegionSize)) -
                                    ((conn.X * (int) Constants.RegionSize)));
                        offset.Y = (((regionConnections.Y * (int) Constants.RegionSize)) -
                                    ((conn.Y * (int) Constants.RegionSize)));

                        Vector3 extents = Vector3.Zero;
                        extents.Y = regionConnections.YEnd + conn.YEnd;
                        extents.X = conn.XEnd + conn.XEnd;

                        m_log.DebugFormat("Scene: {0} to the northwest of Scene{1}.  Offset: {2}.  Extents:{3}",
                                          conn.RegionScene.RegionInfo.RegionName,
                                          regionConnections.RegionScene.RegionInfo.RegionName,
                                          offset, extents);

                        scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, extents);
                            
                        connectedYN = true;
                        break;
                    }
                    */

                    /*
                    //If we're one region over x +y
                    //xxx
                    //xxx
                    //xyx
                    if ((((int)conn.X * (int)Constants.RegionSize)
                        == (regionConnections.X * (int)Constants.RegionSize))
                        && (((int)conn.Y * (int)Constants.RegionSize) - conn.YEnd
                        == (regionConnections.Y * (int)Constants.RegionSize)))
                    {
                        Vector3 offset = Vector3.Zero;
                        offset.X = (((regionConnections.X * (int)Constants.RegionSize)) -
                                    ((conn.X * (int)Constants.RegionSize)));
                        offset.Y = (((regionConnections.Y * (int)Constants.RegionSize)) -
                                    ((conn.Y * (int)Constants.RegionSize)));

                        Vector3 extents = Vector3.Zero;
                        extents.Y = regionConnections.YEnd + conn.YEnd;
                        extents.X = conn.XEnd;

                        m_log.DebugFormat("Scene: {0} to the north of Scene{1}.  Offset: {2}. Extents:{3}",
                                          conn.RegionScene.RegionInfo.RegionName,
                                          regionConnections.RegionScene.RegionInfo.RegionName, offset, extents);

                        scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, extents);
                        connectedYN = true;
                        break;
                    }
                    */

                    /*
                    // If we're one region over -x +y
                    //xxx
                    //xxx
                    //yxx
                    if ((((int)conn.X * (int)Constants.RegionSize) - conn.XEnd
                        == (regionConnections.X * (int)Constants.RegionSize))
                        && (((int)conn.Y * (int)Constants.RegionSize) - conn.YEnd
                        == (regionConnections.Y * (int)Constants.RegionSize)))
                    {
                        Vector3 offset = Vector3.Zero;
                        offset.X = (((regionConnections.X * (int)Constants.RegionSize)) -
                                    ((conn.X * (int)Constants.RegionSize)));
                        offset.Y = (((regionConnections.Y * (int)Constants.RegionSize)) -
                                    ((conn.Y * (int)Constants.RegionSize)));

                        Vector3 extents = Vector3.Zero;
                        extents.Y = regionConnections.YEnd + conn.YEnd;
                        extents.X = conn.XEnd + conn.XEnd;

                        m_log.DebugFormat("Scene: {0} to the northeast of Scene.  Offset: {2}. Extents:{3}",
                                          conn.RegionScene.RegionInfo.RegionName,
                                          regionConnections.RegionScene.RegionInfo.RegionName, offset, extents);

                        scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, extents);


                        connectedYN = true;
                        break;
                    }
                    */

                    /*
                    // If we're one region over -x y
                    //xxx
                    //yxx
                    //xxx
                    if ((((int)conn.X * (int)Constants.RegionSize) - conn.XEnd
                        == (regionConnections.X * (int)Constants.RegionSize))
                        && (((int)conn.Y * (int)Constants.RegionSize)
                        == (regionConnections.Y * (int)Constants.RegionSize)))
                    {
                        Vector3 offset = Vector3.Zero;
                        offset.X = (((regionConnections.X * (int)Constants.RegionSize)) -
                                    ((conn.X * (int)Constants.RegionSize)));
                        offset.Y = (((regionConnections.Y * (int)Constants.RegionSize)) -
                                    ((conn.Y * (int)Constants.RegionSize)));

                        Vector3 extents = Vector3.Zero;
                        extents.Y = regionConnections.YEnd;
                        extents.X = conn.XEnd + conn.XEnd;

                        m_log.DebugFormat("Scene: {0} to the east of Scene{1} Offset: {2}. Extents:{3}",
                                          conn.RegionScene.RegionInfo.RegionName,
                                          regionConnections.RegionScene.RegionInfo.RegionName, offset, extents);

                        scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, extents);

                        connectedYN = true;
                        break;
                    }
                    */

                    /*
                        // If we're one region over -x -y
                        //yxx
                        //xxx
                        //xxx
                        if ((((int)conn.X * (int)Constants.RegionSize) - conn.XEnd
                            == (regionConnections.X * (int)Constants.RegionSize))
                            && (((int)conn.Y * (int)Constants.RegionSize) + conn.YEnd
                            == (regionConnections.Y * (int)Constants.RegionSize)))
                        {
                            Vector3 offset = Vector3.Zero;
                            offset.X = (((regionConnections.X * (int)Constants.RegionSize)) -
                                        ((conn.X * (int)Constants.RegionSize)));
                            offset.Y = (((regionConnections.Y * (int)Constants.RegionSize)) -
                                        ((conn.Y * (int)Constants.RegionSize)));

                            Vector3 extents = Vector3.Zero;
                            extents.Y = regionConnections.YEnd + conn.YEnd;
                            extents.X = conn.XEnd + conn.XEnd;

                            m_log.DebugFormat("Scene: {0} to the northeast of Scene{1} Offset: {2}. Extents:{3}",
                                              conn.RegionScene.RegionInfo.RegionName,
                                              regionConnections.RegionScene.RegionInfo.RegionName, offset, extents);

                            scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, extents);

                            connectedYN = true;
                            break;
                        }
                        */
                    #endregion

                    // If we're one region over +x y
                    //xxx
                    //xxy
                    //xxx


                    if ((((int)conn.X * (int)Constants.RegionSize) + conn.XEnd
                        >= (regionConnections.X * (int)Constants.RegionSize))
                        && (((int)conn.Y * (int)Constants.RegionSize)
                        >= (regionConnections.Y * (int)Constants.RegionSize)))
                    {
                        connectedYN = DoWorkForOneRegionOverPlusXY(conn, regionConnections, scene);
                        break;
                    }

                    // If we're one region over x +y
                    //xyx
                    //xxx
                    //xxx
                    if ((((int)conn.X * (int)Constants.RegionSize)
                        >= (regionConnections.X * (int)Constants.RegionSize))
                        && (((int)conn.Y * (int)Constants.RegionSize) + conn.YEnd
                        >= (regionConnections.Y * (int)Constants.RegionSize)))
                    {
                        connectedYN = DoWorkForOneRegionOverXPlusY(conn, regionConnections, scene);
                        break;
                    }

                    // If we're one region over +x +y
                    //xxy
                    //xxx
                    //xxx
                    if ((((int)conn.X * (int)Constants.RegionSize) + conn.YEnd
                        >= (regionConnections.X * (int)Constants.RegionSize))
                        && (((int)conn.Y * (int)Constants.RegionSize) + conn.YEnd
                        >= (regionConnections.Y * (int)Constants.RegionSize)))
                    {
                        connectedYN = DoWorkForOneRegionOverPlusXPlusY(conn, regionConnections, scene);
                        break;

                    }
                }

                // If !connectYN means that this region is a root region
                if (!connectedYN)
                {
                    DoWorkForRootRegion(regionConnections, scene);

                }
            }
            // Set up infinite borders around the entire AABB of the combined ConnectedRegions
            AdjustLargeRegionBounds();
        }

        private bool DoWorkForOneRegionOverPlusXY(RegionConnections conn, RegionConnections regionConnections, Scene scene)
        {
            Vector3 offset = Vector3.Zero;
            offset.X = (((regionConnections.X * (int)Constants.RegionSize)) -
                        ((conn.X * (int)Constants.RegionSize)));
            offset.Y = (((regionConnections.Y * (int)Constants.RegionSize)) -
                        ((conn.Y * (int)Constants.RegionSize)));

            Vector3 extents = Vector3.Zero;
            extents.Y = conn.YEnd;
            extents.X = conn.XEnd + regionConnections.XEnd;

            conn.UpdateExtents(extents);

            m_log.DebugFormat("Scene: {0} to the west of Scene{1} Offset: {2}. Extents:{3}",
                              conn.RegionScene.RegionInfo.RegionName,
                              regionConnections.RegionScene.RegionInfo.RegionName, offset, extents);

            scene.BordersLocked = true;
            conn.RegionScene.BordersLocked = true;

            RegionData ConnectedRegion = new RegionData();
            ConnectedRegion.Offset = offset;
            ConnectedRegion.RegionId = scene.RegionInfo.originRegionID;
            ConnectedRegion.RegionScene = scene;
            conn.ConnectedRegions.Add(ConnectedRegion);

            // Inform root region Physics about the extents of this region
            conn.RegionScene.PhysicsScene.Combine(null, Vector3.Zero, extents);

            // Inform Child region that it needs to forward it's terrain to the root region
            scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, Vector3.Zero);

            // Extend the borders as appropriate
            lock (conn.RegionScene.EastBorders)
                conn.RegionScene.EastBorders[0].BorderLine.Z += (int)Constants.RegionSize;

            lock (conn.RegionScene.NorthBorders)
                conn.RegionScene.NorthBorders[0].BorderLine.Y += (int)Constants.RegionSize;

            lock (conn.RegionScene.SouthBorders)
                conn.RegionScene.SouthBorders[0].BorderLine.Y += (int)Constants.RegionSize;

            lock (scene.WestBorders)
            {


                scene.WestBorders[0].BorderLine.Z = (int)((scene.RegionInfo.RegionLocX - conn.RegionScene.RegionInfo.RegionLocX) * (int)Constants.RegionSize); //auto teleport West

                // Trigger auto teleport to root region
                scene.WestBorders[0].TriggerRegionX = conn.RegionScene.RegionInfo.RegionLocX;
                scene.WestBorders[0].TriggerRegionY = conn.RegionScene.RegionInfo.RegionLocY;
            }

            // Reset Terrain..  since terrain loads before we get here, we need to load 
            // it again so it loads in the root region

            scene.PhysicsScene.SetTerrain(scene.Heightmap.GetFloatsSerialised());

            // Unlock borders
            conn.RegionScene.BordersLocked = false;
            scene.BordersLocked = false;

            // Create a client event forwarder and add this region's events to the root region.
            if (conn.ClientEventForwarder != null)
                conn.ClientEventForwarder.AddSceneToEventForwarding(scene);

            return true;
        }

        private bool DoWorkForOneRegionOverXPlusY(RegionConnections conn, RegionConnections regionConnections, Scene scene)
        {
            Vector3 offset = Vector3.Zero;
            offset.X = (((regionConnections.X * (int)Constants.RegionSize)) -
                        ((conn.X * (int)Constants.RegionSize)));
            offset.Y = (((regionConnections.Y * (int)Constants.RegionSize)) -
                        ((conn.Y * (int)Constants.RegionSize)));

            Vector3 extents = Vector3.Zero;
            extents.Y = regionConnections.YEnd + conn.YEnd;
            extents.X = conn.XEnd;
            conn.UpdateExtents(extents);

            scene.BordersLocked = true;
            conn.RegionScene.BordersLocked = true;

            RegionData ConnectedRegion = new RegionData();
            ConnectedRegion.Offset = offset;
            ConnectedRegion.RegionId = scene.RegionInfo.originRegionID;
            ConnectedRegion.RegionScene = scene;
            conn.ConnectedRegions.Add(ConnectedRegion);

            m_log.DebugFormat("Scene: {0} to the northeast of Scene{1} Offset: {2}. Extents:{3}",
                             conn.RegionScene.RegionInfo.RegionName,
                             regionConnections.RegionScene.RegionInfo.RegionName, offset, extents);

            conn.RegionScene.PhysicsScene.Combine(null, Vector3.Zero, extents);
            scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, Vector3.Zero);

            lock (conn.RegionScene.NorthBorders)
                conn.RegionScene.NorthBorders[0].BorderLine.Z += (int)Constants.RegionSize;
            lock (conn.RegionScene.EastBorders)
                conn.RegionScene.EastBorders[0].BorderLine.Y += (int)Constants.RegionSize;
            lock (conn.RegionScene.WestBorders)
                conn.RegionScene.WestBorders[0].BorderLine.Y += (int)Constants.RegionSize;
            lock (scene.SouthBorders)
            {
                scene.SouthBorders[0].BorderLine.Z = (int)((scene.RegionInfo.RegionLocY - conn.RegionScene.RegionInfo.RegionLocY) * (int)Constants.RegionSize); //auto teleport south
                scene.SouthBorders[0].TriggerRegionX = conn.RegionScene.RegionInfo.RegionLocX;
                scene.SouthBorders[0].TriggerRegionY = conn.RegionScene.RegionInfo.RegionLocY;
            }

            // Reset Terrain..  since terrain normally loads first.
            //conn.RegionScene.PhysicsScene.SetTerrain(conn.RegionScene.Heightmap.GetFloatsSerialised());
            scene.PhysicsScene.SetTerrain(scene.Heightmap.GetFloatsSerialised());
            //conn.RegionScene.PhysicsScene.SetTerrain(conn.RegionScene.Heightmap.GetFloatsSerialised());

            scene.BordersLocked = false;
            conn.RegionScene.BordersLocked = false;
            if (conn.ClientEventForwarder != null)
                conn.ClientEventForwarder.AddSceneToEventForwarding(scene);
            return true;
        }

        private bool DoWorkForOneRegionOverPlusXPlusY(RegionConnections conn, RegionConnections regionConnections, Scene scene)
        {
            Vector3 offset = Vector3.Zero;
            offset.X = (((regionConnections.X * (int)Constants.RegionSize)) -
                        ((conn.X * (int)Constants.RegionSize)));
            offset.Y = (((regionConnections.Y * (int)Constants.RegionSize)) -
                        ((conn.Y * (int)Constants.RegionSize)));

            Vector3 extents = Vector3.Zero;
            extents.Y = regionConnections.YEnd + conn.YEnd;
            extents.X = regionConnections.XEnd + conn.XEnd;
            conn.UpdateExtents(extents);

            scene.BordersLocked = true;
            conn.RegionScene.BordersLocked = true;

            RegionData ConnectedRegion = new RegionData();
            ConnectedRegion.Offset = offset;
            ConnectedRegion.RegionId = scene.RegionInfo.originRegionID;
            ConnectedRegion.RegionScene = scene;

            conn.ConnectedRegions.Add(ConnectedRegion);

            m_log.DebugFormat("Scene: {0} to the NorthEast of Scene{1} Offset: {2}. Extents:{3}",
                             conn.RegionScene.RegionInfo.RegionName,
                             regionConnections.RegionScene.RegionInfo.RegionName, offset, extents);

            conn.RegionScene.PhysicsScene.Combine(null, Vector3.Zero, extents);
            scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset, Vector3.Zero);
            lock (conn.RegionScene.NorthBorders)
            {
                if (conn.RegionScene.NorthBorders.Count == 1)// &&  2)
                {
                    //compound border
                    // already locked above
                    conn.RegionScene.NorthBorders[0].BorderLine.Z += (int)Constants.RegionSize;

                    lock (conn.RegionScene.EastBorders)
                        conn.RegionScene.EastBorders[0].BorderLine.Y += (int)Constants.RegionSize;
                    lock (conn.RegionScene.WestBorders)
                        conn.RegionScene.WestBorders[0].BorderLine.Y += (int)Constants.RegionSize;
                }
            }

            lock (scene.SouthBorders)
            {
                scene.SouthBorders[0].BorderLine.Z = (int)((scene.RegionInfo.RegionLocY - conn.RegionScene.RegionInfo.RegionLocY) * (int)Constants.RegionSize); //auto teleport south
                scene.SouthBorders[0].TriggerRegionX = conn.RegionScene.RegionInfo.RegionLocX;
                scene.SouthBorders[0].TriggerRegionY = conn.RegionScene.RegionInfo.RegionLocY;
            }

            lock (conn.RegionScene.EastBorders)
            {
                if (conn.RegionScene.EastBorders.Count == 1)// && conn.RegionScene.EastBorders.Count == 2)
                {

                    conn.RegionScene.EastBorders[0].BorderLine.Z += (int)Constants.RegionSize;
                    lock (conn.RegionScene.NorthBorders)
                        conn.RegionScene.NorthBorders[0].BorderLine.Y += (int)Constants.RegionSize;
                    lock (conn.RegionScene.SouthBorders)
                        conn.RegionScene.SouthBorders[0].BorderLine.Y += (int)Constants.RegionSize;


                }
            }

            lock (scene.WestBorders)
            {
                scene.WestBorders[0].BorderLine.Z = (int)((scene.RegionInfo.RegionLocX - conn.RegionScene.RegionInfo.RegionLocX) * (int)Constants.RegionSize); //auto teleport West
                scene.WestBorders[0].TriggerRegionX = conn.RegionScene.RegionInfo.RegionLocX;
                scene.WestBorders[0].TriggerRegionY = conn.RegionScene.RegionInfo.RegionLocY;
            }

            /*
                                    else
                                    {
                                        conn.RegionScene.NorthBorders[0].BorderLine.Z += (int)Constants.RegionSize;
                                        conn.RegionScene.EastBorders[0].BorderLine.Y += (int)Constants.RegionSize;
                                        conn.RegionScene.WestBorders[0].BorderLine.Y += (int)Constants.RegionSize;
                                        scene.SouthBorders[0].BorderLine.Z += (int)Constants.RegionSize; //auto teleport south
                                    }
            */


            // Reset Terrain..  since terrain normally loads first.
            //conn.RegionScene.PhysicsScene.SetTerrain(conn.RegionScene.Heightmap.GetFloatsSerialised());
            scene.PhysicsScene.SetTerrain(scene.Heightmap.GetFloatsSerialised());
            //conn.RegionScene.PhysicsScene.SetTerrain(conn.RegionScene.Heightmap.GetFloatsSerialised());
            scene.BordersLocked = false;
            conn.RegionScene.BordersLocked = false;

            if (conn.ClientEventForwarder != null)
                conn.ClientEventForwarder.AddSceneToEventForwarding(scene);

            return true;

            //scene.PhysicsScene.Combine(conn.RegionScene.PhysicsScene, offset,extents);

        }

        private void DoWorkForRootRegion(RegionConnections regionConnections, Scene scene)
        {
            RegionData rdata = new RegionData();
            rdata.Offset = Vector3.Zero;
            rdata.RegionId = scene.RegionInfo.originRegionID;
            rdata.RegionScene = scene;
            // save it's land channel
            regionConnections.RegionLandChannel = scene.LandChannel;

            // Substitue our landchannel
            RegionCombinerLargeLandChannel lnd = new RegionCombinerLargeLandChannel(rdata, scene.LandChannel,
                                                            regionConnections.ConnectedRegions);
            scene.LandChannel = lnd;
            // Forward the permissions modules of each of the connected regions to the root region
            lock (m_regions)
            {
                foreach (RegionData r in regionConnections.ConnectedRegions)
                {
                    ForwardPermissionRequests(regionConnections, r.RegionScene);
                }
            }
            // Create the root region's Client Event Forwarder
            regionConnections.ClientEventForwarder = new RegionCombinerClientEventForwarder(regionConnections);

            // Sets up the CoarseLocationUpdate forwarder for this root region
            scene.EventManager.OnNewPresence += SetCourseLocationDelegate;

            // Adds this root region to a dictionary of regions that are connectable
            m_regions.Add(scene.RegionInfo.originRegionID, regionConnections);
        }

        private void SetCourseLocationDelegate(ScenePresence presence)
        {
            presence.SetSendCourseLocationMethod(SendCourseLocationUpdates);
        }

        // This delegate was refactored for non-combined regions.
        // This combined region version will not use the pre-compiled lists of locations and ids
        private void SendCourseLocationUpdates(UUID sceneId, ScenePresence presence, List<Vector3> coarseLocations, List<UUID> avatarUUIDs)
        {
            RegionConnections connectiondata = null; 
            lock (m_regions)
            {
                if (m_regions.ContainsKey(sceneId))
                    connectiondata = m_regions[sceneId];
                else
                    return;
            }

            List<Vector3> CoarseLocations = new List<Vector3>();
            List<UUID> AvatarUUIDs = new List<UUID>();
            connectiondata.RegionScene.ForEachRootScenePresence(delegate(ScenePresence sp)
            {
                if (sp.UUID != presence.UUID)
                {
                    if (sp.ParentID != 0)
                    {
                        // sitting avatar
                        SceneObjectPart sop = connectiondata.RegionScene.GetSceneObjectPart(sp.ParentID);
                        if (sop != null)
                        {
                            CoarseLocations.Add(sop.AbsolutePosition + sp.AbsolutePosition);
                            AvatarUUIDs.Add(sp.UUID);
                        }
                        else
                        {
                            // we can't find the parent..  ! arg!
                            CoarseLocations.Add(sp.AbsolutePosition);
                            AvatarUUIDs.Add(sp.UUID);
                        }
                    }
                    else
                    {
                        CoarseLocations.Add(sp.AbsolutePosition);
                        AvatarUUIDs.Add(sp.UUID);
                    }
                }
            });
            DistributeCourseLocationUpdates(CoarseLocations, AvatarUUIDs, connectiondata, presence);
        }

        private void DistributeCourseLocationUpdates(List<Vector3> locations, List<UUID> uuids, 
                                                     RegionConnections connectiondata, ScenePresence rootPresence)
        {
            RegionData[] rdata = connectiondata.ConnectedRegions.ToArray();
            //List<IClientAPI> clients = new List<IClientAPI>();
            Dictionary<Vector2, RegionCourseLocationStruct> updates = new Dictionary<Vector2, RegionCourseLocationStruct>();
            
            // Root Region entry
            RegionCourseLocationStruct rootupdatedata = new RegionCourseLocationStruct();
            rootupdatedata.Locations = new List<Vector3>();
            rootupdatedata.Uuids = new List<UUID>();
            rootupdatedata.Offset = Vector2.Zero;

            rootupdatedata.UserAPI = rootPresence.ControllingClient;

            if (rootupdatedata.UserAPI != null)
                updates.Add(Vector2.Zero, rootupdatedata);

            //Each Region needs an entry or we will end up with dead minimap dots
            foreach (RegionData regiondata in rdata)
            {
                Vector2 offset = new Vector2(regiondata.Offset.X, regiondata.Offset.Y);
                RegionCourseLocationStruct updatedata = new RegionCourseLocationStruct();
                updatedata.Locations = new List<Vector3>();
                updatedata.Uuids = new List<UUID>();
                updatedata.Offset = offset;

                if (offset == Vector2.Zero)
                    updatedata.UserAPI = rootPresence.ControllingClient;
                else
                    updatedata.UserAPI = LocateUsersChildAgentIClientAPI(offset, rootPresence.UUID, rdata);

                if (updatedata.UserAPI != null)
                    updates.Add(offset, updatedata);
            }

            // go over the locations and assign them to an IClientAPI
            for (int i = 0; i < locations.Count; i++)
            //{locations[i]/(int) Constants.RegionSize;
            {
                Vector3 pPosition = new Vector3((int)locations[i].X / (int)Constants.RegionSize, 
                                                (int)locations[i].Y / (int)Constants.RegionSize, locations[i].Z);
                Vector2 offset = new Vector2(pPosition.X*(int) Constants.RegionSize,
                                             pPosition.Y*(int) Constants.RegionSize);
                
                if (!updates.ContainsKey(offset))
                {
                    // This shouldn't happen
                    RegionCourseLocationStruct updatedata = new RegionCourseLocationStruct();
                    updatedata.Locations = new List<Vector3>();
                    updatedata.Uuids = new List<UUID>();
                    updatedata.Offset = offset;
                    
                    if (offset == Vector2.Zero)
                        updatedata.UserAPI = rootPresence.ControllingClient;
                    else 
                        updatedata.UserAPI = LocateUsersChildAgentIClientAPI(offset, rootPresence.UUID, rdata);

                    updates.Add(offset,updatedata);
                }
                
                updates[offset].Locations.Add(locations[i]);
                updates[offset].Uuids.Add(uuids[i]);
            }

            // Send out the CoarseLocationupdates from their respective client connection based on where the avatar is
            foreach (Vector2 offset in updates.Keys)
            {
                if (updates[offset].UserAPI != null)
                {
                    updates[offset].UserAPI.SendCoarseLocationUpdate(updates[offset].Uuids,updates[offset].Locations);
                }
            }
        }

        /// <summary>
        /// Locates a the Client of a particular region in an Array of RegionData based on offset
        /// </summary>
        /// <param name="offset"></param>
        /// <param name="uUID"></param>
        /// <param name="rdata"></param>
        /// <returns>IClientAPI or null</returns>
        private IClientAPI LocateUsersChildAgentIClientAPI(Vector2 offset, UUID uUID, RegionData[] rdata)
        {
            IClientAPI returnclient = null;
            foreach (RegionData r in rdata)
            {
                if (r.Offset.X == offset.X && r.Offset.Y == offset.Y)
                {
                    return r.RegionScene.SceneGraph.GetControllingClient(uUID);
                }
            }

            return returnclient;
        }

        public void PostInitialise()
        {
        }
        
//        /// <summary>
//        /// TODO:
//        /// </summary>
//        /// <param name="rdata"></param>
//        public void UnCombineRegion(RegionData rdata)
//        {
//            lock (m_regions)
//            {
//                if (m_regions.ContainsKey(rdata.RegionId))
//                {
//                    // uncombine root region and virtual regions
//                }
//                else
//                {
//                    foreach (RegionConnections r in m_regions.Values)
//                    {
//                        foreach (RegionData rd in r.ConnectedRegions)
//                        {
//                            if (rd.RegionId == rdata.RegionId)
//                            {
//                                // uncombine virtual region
//                            }
//                        }
//                    }
//                }
//            }
//        }

        // Create a set of infinite borders around the whole aabb of the combined island.
        private void AdjustLargeRegionBounds()
        {
            lock (m_regions)
            {
                foreach (RegionConnections rconn in m_regions.Values)
                {
                    Vector3 offset = Vector3.Zero;
                    rconn.RegionScene.BordersLocked = true;
                    foreach (RegionData rdata in rconn.ConnectedRegions)
                    {
                        if (rdata.Offset.X > offset.X) offset.X = rdata.Offset.X;
                        if (rdata.Offset.Y > offset.Y) offset.Y = rdata.Offset.Y;
                    }

                    lock (rconn.RegionScene.NorthBorders)
                    {
                        Border northBorder = null;
                        // If we don't already have an infinite border, create one.
                        if (!TryGetInfiniteBorder(rconn.RegionScene.NorthBorders, out northBorder))
                        {
                            northBorder = new Border();
                            rconn.RegionScene.NorthBorders.Add(northBorder);
                        }
                        
                        northBorder.BorderLine = new Vector3(float.MinValue, float.MaxValue,
                                                             offset.Y + (int) Constants.RegionSize); //<---
                        northBorder.CrossDirection = Cardinals.N;
                    }

                    lock (rconn.RegionScene.SouthBorders)
                    {
                        Border southBorder = null;
                        // If we don't already have an infinite border, create one.
                        if (!TryGetInfiniteBorder(rconn.RegionScene.SouthBorders, out southBorder))
                        {
                            southBorder = new Border();
                            rconn.RegionScene.SouthBorders.Add(southBorder);
                        }
                        southBorder.BorderLine = new Vector3(float.MinValue, float.MaxValue, 0); //--->
                        southBorder.CrossDirection = Cardinals.S;
                    }

                    lock (rconn.RegionScene.EastBorders)
                    {
                        Border eastBorder = null;
                        // If we don't already have an infinite border, create one.
                        if (!TryGetInfiniteBorder(rconn.RegionScene.EastBorders, out eastBorder))
                        {
                            eastBorder = new Border();
                            rconn.RegionScene.EastBorders.Add(eastBorder);
                        }
                        eastBorder.BorderLine = new Vector3(float.MinValue, float.MaxValue, offset.X + (int)Constants.RegionSize);
                        //<---
                        eastBorder.CrossDirection = Cardinals.E;
                    }

                    lock (rconn.RegionScene.WestBorders)
                    {
                        Border westBorder = null;
                        // If we don't already have an infinite border, create one.
                        if (!TryGetInfiniteBorder(rconn.RegionScene.WestBorders, out westBorder))
                        {
                            westBorder = new Border();
                            rconn.RegionScene.WestBorders.Add(westBorder);

                        }
                        westBorder.BorderLine = new Vector3(float.MinValue, float.MaxValue, 0); //--->
                        westBorder.CrossDirection = Cardinals.W;
                    }

                    rconn.RegionScene.BordersLocked = false;
                }
            }
        }

        /// <summary>
        /// Try and get an Infinite border out of a listT of borders
        /// </summary>
        /// <param name="borders"></param>
        /// <param name="oborder"></param>
        /// <returns></returns>
        public static bool TryGetInfiniteBorder(List<Border> borders, out Border oborder)
        {
            // Warning! Should be locked before getting here!
            foreach (Border b in borders)
            {
                if (b.BorderLine.X == float.MinValue && b.BorderLine.Y == float.MaxValue)
                {
                    oborder = b;
                    return true;
                }
            }
            oborder = null;
            return false;
        }
       
        public RegionData GetRegionFromPosition(Vector3 pPosition)
        {
            pPosition = pPosition/(int) Constants.RegionSize;
            int OffsetX = (int) pPosition.X;
            int OffsetY = (int) pPosition.Y;
            foreach (RegionConnections regConn in m_regions.Values)
            {
                foreach (RegionData reg in regConn.ConnectedRegions)
                {
                    if (reg.Offset.X == OffsetX && reg.Offset.Y == OffsetY)
                        return reg;
                }
            }
            return new RegionData();
        }

        public void ForwardPermissionRequests(RegionConnections BigRegion, Scene VirtualRegion)
        {
            if (BigRegion.PermissionModule == null)
                BigRegion.PermissionModule = new RegionCombinerPermissionModule(BigRegion.RegionScene);

            VirtualRegion.Permissions.OnBypassPermissions += BigRegion.PermissionModule.BypassPermissions;
            VirtualRegion.Permissions.OnSetBypassPermissions += BigRegion.PermissionModule.SetBypassPermissions;
            VirtualRegion.Permissions.OnPropagatePermissions += BigRegion.PermissionModule.PropagatePermissions;
            VirtualRegion.Permissions.OnGenerateClientFlags += BigRegion.PermissionModule.GenerateClientFlags;
            VirtualRegion.Permissions.OnAbandonParcel += BigRegion.PermissionModule.CanAbandonParcel;
            VirtualRegion.Permissions.OnReclaimParcel += BigRegion.PermissionModule.CanReclaimParcel;
            VirtualRegion.Permissions.OnDeedParcel += BigRegion.PermissionModule.CanDeedParcel;
            VirtualRegion.Permissions.OnDeedObject += BigRegion.PermissionModule.CanDeedObject;
            VirtualRegion.Permissions.OnIsGod += BigRegion.PermissionModule.IsGod;
            VirtualRegion.Permissions.OnDuplicateObject += BigRegion.PermissionModule.CanDuplicateObject;
            VirtualRegion.Permissions.OnDeleteObject += BigRegion.PermissionModule.CanDeleteObject; //MAYBE FULLY IMPLEMENTED
            VirtualRegion.Permissions.OnEditObject += BigRegion.PermissionModule.CanEditObject; //MAYBE FULLY IMPLEMENTED
            VirtualRegion.Permissions.OnEditParcelProperties += BigRegion.PermissionModule.CanEditParcelProperties; //MAYBE FULLY IMPLEMENTED
            VirtualRegion.Permissions.OnInstantMessage += BigRegion.PermissionModule.CanInstantMessage;
            VirtualRegion.Permissions.OnInventoryTransfer += BigRegion.PermissionModule.CanInventoryTransfer; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnIssueEstateCommand += BigRegion.PermissionModule.CanIssueEstateCommand; //FULLY IMPLEMENTED
            VirtualRegion.Permissions.OnMoveObject += BigRegion.PermissionModule.CanMoveObject; //MAYBE FULLY IMPLEMENTED
            VirtualRegion.Permissions.OnObjectEntry += BigRegion.PermissionModule.CanObjectEntry;
            VirtualRegion.Permissions.OnReturnObjects += BigRegion.PermissionModule.CanReturnObjects; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnRezObject += BigRegion.PermissionModule.CanRezObject; //MAYBE FULLY IMPLEMENTED
            VirtualRegion.Permissions.OnRunConsoleCommand += BigRegion.PermissionModule.CanRunConsoleCommand;
            VirtualRegion.Permissions.OnRunScript += BigRegion.PermissionModule.CanRunScript; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnCompileScript += BigRegion.PermissionModule.CanCompileScript;
            VirtualRegion.Permissions.OnSellParcel += BigRegion.PermissionModule.CanSellParcel;
            VirtualRegion.Permissions.OnTakeObject += BigRegion.PermissionModule.CanTakeObject;
            VirtualRegion.Permissions.OnTakeCopyObject += BigRegion.PermissionModule.CanTakeCopyObject;
            VirtualRegion.Permissions.OnTerraformLand += BigRegion.PermissionModule.CanTerraformLand;
            VirtualRegion.Permissions.OnLinkObject += BigRegion.PermissionModule.CanLinkObject; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnDelinkObject += BigRegion.PermissionModule.CanDelinkObject; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnBuyLand += BigRegion.PermissionModule.CanBuyLand; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnViewNotecard += BigRegion.PermissionModule.CanViewNotecard; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnViewScript += BigRegion.PermissionModule.CanViewScript; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnEditNotecard += BigRegion.PermissionModule.CanEditNotecard; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnEditScript += BigRegion.PermissionModule.CanEditScript; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnCreateObjectInventory += BigRegion.PermissionModule.CanCreateObjectInventory; //NOT IMPLEMENTED HERE 
            VirtualRegion.Permissions.OnEditObjectInventory += BigRegion.PermissionModule.CanEditObjectInventory;//MAYBE FULLY IMPLEMENTED
            VirtualRegion.Permissions.OnCopyObjectInventory += BigRegion.PermissionModule.CanCopyObjectInventory; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnDeleteObjectInventory += BigRegion.PermissionModule.CanDeleteObjectInventory; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnResetScript += BigRegion.PermissionModule.CanResetScript;
            VirtualRegion.Permissions.OnCreateUserInventory += BigRegion.PermissionModule.CanCreateUserInventory; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnCopyUserInventory += BigRegion.PermissionModule.CanCopyUserInventory; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnEditUserInventory += BigRegion.PermissionModule.CanEditUserInventory; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnDeleteUserInventory += BigRegion.PermissionModule.CanDeleteUserInventory; //NOT YET IMPLEMENTED
            VirtualRegion.Permissions.OnTeleport += BigRegion.PermissionModule.CanTeleport; //NOT YET IMPLEMENTED
        }

        #region console commands
        public void FixPhantoms(string module, string[] cmdparams)
        {
        List<Scene> scenes = new List<Scene>(m_startingScenes.Values);
            foreach (Scene s in scenes)
            {
                s.ForEachSOG(delegate(SceneObjectGroup e)
                {
                    e.AbsolutePosition = e.AbsolutePosition;
                }
                );
            }
        }
        #endregion
    }
}
