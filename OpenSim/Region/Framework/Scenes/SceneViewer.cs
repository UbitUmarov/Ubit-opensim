﻿/*
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
using OpenMetaverse;
using log4net;
using OpenSim.Framework;
using OpenSim.Framework.Client;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes.Types;

namespace OpenSim.Region.Framework.Scenes
{
    public class SceneViewer : ISceneViewer
    {
        /// <summary>
        /// Is this scene viewer enabled?
        /// </summary>
        private bool IsEnabled { get; set; }

        /// <summary>
        /// The scene presence serviced by this viewer.
        /// </summary>
        protected ScenePresence m_presence;

        /// <summary>
        /// The queue of parts for which we need to send out updates.
        /// </summary>
        protected UpdateQueue m_partsUpdateQueue = new UpdateQueue();

        /// <summary>
        /// The queue of objects for which we need to send out updates.
        /// </summary>
        protected Queue<SceneObjectGroup> m_pendingObjects;

        /// <summary>
        /// The last update assocated with a given part update.
        /// </summary>
        protected Dictionary<UUID, ScenePartUpdate> m_updateTimes = new Dictionary<UUID, ScenePartUpdate>();

        public SceneViewer(ScenePresence presence)
        {
            m_presence = presence;
            IsEnabled = true;
        }

        public void QueuePartForUpdate(SceneObjectPart part)
        {
            if (!IsEnabled)
                return;

            lock (m_partsUpdateQueue)
            {
                m_partsUpdateQueue.Enqueue(part);
            }
        }

        public void SendPrimUpdates()
        {
            if (m_pendingObjects == null)
            {
                m_pendingObjects = new Queue<SceneObjectGroup>();

                lock (m_pendingObjects)
                {
                    EntityBase[] entities = m_presence.Scene.Entities.GetEntities();
                    foreach (EntityBase e in entities)
                    {
                        if (e != null && e is SceneObjectGroup)
                            m_pendingObjects.Enqueue((SceneObjectGroup)e);
                    }
                }
            }

            lock (m_pendingObjects)
            {
                // We must do this under lock so that we don't suffer a race condition if another thread closes the
                // viewer
                if (!IsEnabled)
                    return;

                while (m_pendingObjects != null && m_pendingObjects.Count > 0)
                {
                    SceneObjectGroup g = m_pendingObjects.Dequeue();
                     // Yes, this can really happen
                     if (g == null)
                        continue;

                    // This is where we should check for draw distance
                    // do culling and stuff. Problem with that is that until
                    // we recheck in movement, that won't work right.
                    // So it's not implemented now.
                    //

                    // Don't even queue if we have sent this one
                    //
                    if (!m_updateTimes.ContainsKey(g.UUID))
                        g.ScheduleFullUpdateToAvatar(m_presence);
                }

                while (m_partsUpdateQueue.Count > 0)
                {
                    SceneObjectPart part = m_partsUpdateQueue.Dequeue();
                    
                    if (part.ParentGroup.IsDeleted)
                        continue;
                    
                    if (m_updateTimes.ContainsKey(part.UUID))
                    {
                        ScenePartUpdate update = m_updateTimes[part.UUID];

                        // We deal with the possibility that two updates occur at
                        // the same unix time at the update point itself.
                        if ((update.LastFullUpdateTime < part.TimeStampFull) || part.ParentGroup.IsAttachment)
                        {
    //                            m_log.DebugFormat(
    //                                "[SCENE PRESENCE]: Fully   updating prim {0}, {1} - part timestamp {2}",
    //                                part.Name, part.UUID, part.TimeStampFull);

                            part.SendFullUpdate(m_presence.ControllingClient,
                                   m_presence.GenerateClientFlags(part.UUID));

                            // We'll update to the part's timestamp rather than
                            // the current time to avoid the race condition
                            // whereby the next tick occurs while we are doing
                            // this update. If this happened, then subsequent
                            // updates which occurred on the same tick or the
                            // next tick of the last update would be ignored.
                            update.LastFullUpdateTime = part.TimeStampFull;
                        }
                        else if (update.LastTerseUpdateTime <= part.TimeStampTerse)
                        {
    //                            m_log.DebugFormat(
    //                                "[SCENE PRESENCE]: Tersely updating prim {0}, {1} - part timestamp {2}",
    //                                part.Name, part.UUID, part.TimeStampTerse);

                            part.SendTerseUpdateToClient(m_presence.ControllingClient);

                            update.LastTerseUpdateTime = part.TimeStampTerse;
                        }
                    }
                    else
                    {
                        //never been sent to client before so do full update
                        ScenePartUpdate update = new ScenePartUpdate();
                        update.FullID = part.UUID;
                        update.LastFullUpdateTime = part.TimeStampFull;
                        m_updateTimes.Add(part.UUID, update);

                        // Attachment handling
                        //
                        if (part.ParentGroup.RootPart.Shape.PCode == 9 && part.ParentGroup.RootPart.Shape.State != 0)
                        {
                            if (part != part.ParentGroup.RootPart)
                                continue;

                            part.ParentGroup.SendFullUpdateToClient(m_presence.ControllingClient);
                            continue;
                        }

                        part.SendFullUpdate(m_presence.ControllingClient,
                                m_presence.GenerateClientFlags(part.UUID));
                    }
                }
            }
        }

//        public void Reset()
//        {
//            if (m_pendingObjects == null)
//                return;
//
//            lock (m_pendingObjects)
//            {
//                if (m_pendingObjects != null)
//                {
//                    m_pendingObjects.Clear();
//                    m_pendingObjects = null;
//                }
//            }
//        }

        public void Close()
        {
            lock (m_pendingObjects)
            {
                // We perform this under the m_pendingObjects lock in order to avoid a race condition with another
                // thread on SendPrimUpdates()
                IsEnabled = false;

                lock (m_updateTimes)
                {
                    m_updateTimes.Clear();
                }

                lock (m_partsUpdateQueue)
                {
                    m_partsUpdateQueue.Clear();
                }
            }
        }

        public int GetPendingObjectsCount()
        {
            if (m_pendingObjects != null)
                lock (m_pendingObjects)
                    return m_pendingObjects.Count;

            return 0;
        }

        public class ScenePartUpdate
        {
            public UUID FullID;
            public uint LastFullUpdateTime;
            public uint LastTerseUpdateTime;

            public ScenePartUpdate()
            {
                FullID = UUID.Zero;
                LastFullUpdateTime = 0;
                LastTerseUpdateTime = 0;
            }
        }
    }
}
