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
using System.Collections;
using System.Collections.Generic;
using Nini.Config;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;

// using log4net;
// using System.Reflection;


/*****************************************************
 *
 * WorldCommModule
 *
 *
 * Holding place for world comms - basically llListen
 * function implementation.
 *
 * lLListen(integer channel, string name, key id, string msg)
 * The name, id, and msg arguments specify the filtering
 * criteria. You can pass the empty string
 * (or NULL_KEY for id) for these to set a completely
 * open filter; this causes the listen() event handler to be
 * invoked for all chat on the channel. To listen only
 * for chat spoken by a specific object or avatar,
 * specify the name and/or id arguments. To listen
 * only for a specific command, specify the
 * (case-sensitive) msg argument. If msg is not empty,
 * listener will only hear strings which are exactly equal
 * to msg. You can also use all the arguments to establish
 * the most restrictive filtering criteria.
 *
 * It might be useful for each listener to maintain a message
 * digest, with a list of recent messages by UUID.  This can
 * be used to prevent in-world repeater loops.  However, the
 * linden functions do not have this capability, so for now
 * thats the way it works.
 * Instead it blocks messages originating from the same prim.
 * (not Object!)
 *
 * For LSL compliance, note the following:
 * (Tested again 1.21.1 on May 2, 2008)
 * 1. 'id' has to be parsed into a UUID. None-UUID keys are
 *    to be replaced by the ZeroID key. (Well, TryParse does
 *    that for us.
 * 2. Setting up an listen event from the same script, with the
 *    same filter settings (including step 1), returns the same
 *    handle as the original filter.
 * 3. (TODO) handles should be script-local. Starting from 1.
 *    Might be actually easier to map the global handle into
 *    script-local handle in the ScriptEngine. Not sure if its
 *    worth the effort tho.
 *
 * **************************************************/

namespace OpenSim.Region.CoreModules.Scripting.WorldComm
{
    public class WorldCommModule : IRegionModule, IWorldComm
    {
        // private static readonly ILog m_log =
        //     LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private ListenerManager m_listenerManager;
        private Queue m_pending;
        private Queue m_pendingQ;
        private Scene m_scene;
        private int m_whisperdistance = 10;
        private int m_saydistance = 20;
        private int m_shoutdistance = 100;

        #region IRegionModule Members

        public void Initialise(Scene scene, IConfigSource config)
        {
            // wrap this in a try block so that defaults will work if
            // the config file doesn't specify otherwise.
            int maxlisteners = 1000;
            int maxhandles = 64;
            try
            {
                m_whisperdistance = config.Configs["Chat"].GetInt("whisper_distance", m_whisperdistance);
                m_saydistance = config.Configs["Chat"].GetInt("say_distance", m_saydistance);
                m_shoutdistance = config.Configs["Chat"].GetInt("shout_distance", m_shoutdistance);
                maxlisteners = config.Configs["LL-Functions"].GetInt("max_listens_per_region", maxlisteners);
                maxhandles = config.Configs["LL-Functions"].GetInt("max_listens_per_script", maxhandles);
            }
            catch (Exception)
            {
            }
            if (maxlisteners < 1) maxlisteners = int.MaxValue;
            if (maxhandles < 1) maxhandles = int.MaxValue;

            m_scene = scene;
            m_scene.RegisterModuleInterface<IWorldComm>(this);
            m_listenerManager = new ListenerManager(maxlisteners, maxhandles);
            m_scene.EventManager.OnChatFromClient += DeliverClientMessage;
            m_scene.EventManager.OnChatBroadcast += DeliverClientMessage;
            m_pendingQ = new Queue();
            m_pending = Queue.Synchronized(m_pendingQ);
        }

        public void PostInitialise()
        {
        }

        public void Close()
        {
        }

        public string Name
        {
            get { return "WorldCommModule"; }
        }

        public bool IsSharedModule
        {
            get { return false; }
        }

        #endregion

        #region IWorldComm Members

        /// <summary>
        /// Create a listen event callback with the specified filters.
        /// The parameters localID,itemID are needed to uniquely identify
        /// the script during 'peek' time. Parameter hostID is needed to
        /// determine the position of the script.
        /// </summary>
        /// <param name="localID">localID of the script engine</param>
        /// <param name="itemID">UUID of the script engine</param>
        /// <param name="hostID">UUID of the SceneObjectPart</param>
        /// <param name="channel">channel to listen on</param>
        /// <param name="name">name to filter on</param>
        /// <param name="id">key to filter on (user given, could be totally faked)</param>
        /// <param name="msg">msg to filter on</param>
        /// <returns>number of the scripts handle</returns>
        public int Listen(uint localID, UUID itemID, UUID hostID, int channel, string name, UUID id, string msg)
        {
            return m_listenerManager.AddListener(localID, itemID, hostID, channel, name, id, msg);
        }

        /// <summary>
        /// Sets the listen event with handle as active (active = TRUE) or inactive (active = FALSE).
        /// The handle used is returned from Listen()
        /// </summary>
        /// <param name="itemID">UUID of the script engine</param>
        /// <param name="handle">handle returned by Listen()</param>
        /// <param name="active">temp. activate or deactivate the Listen()</param>
        public void ListenControl(UUID itemID, int handle, int active)
        {
            if (active == 1)
                m_listenerManager.Activate(itemID, handle);
            else if (active == 0)
                m_listenerManager.Dectivate(itemID, handle);
        }

        /// <summary>
        /// Removes the listen event callback with handle
        /// </summary>
        /// <param name="itemID">UUID of the script engine</param>
        /// <param name="handle">handle returned by Listen()</param>
        public void ListenRemove(UUID itemID, int handle)
        {
            m_listenerManager.Remove(itemID, handle);
        }

        /// <summary>
        /// Removes all listen event callbacks for the given itemID
        /// (script engine)
        /// </summary>
        /// <param name="itemID">UUID of the script engine</param>
        public void DeleteListener(UUID itemID)
        {
            m_listenerManager.DeleteListener(itemID);
        }


        protected static Vector3 CenterOfRegion = new Vector3(128, 128, 20);

        public void DeliverMessage(ChatTypeEnum type, int channel, string name, UUID id, string msg)
        {
            Vector3 position;
            SceneObjectPart source;
            ScenePresence avatar;

            if ((source = m_scene.GetSceneObjectPart(id)) != null)
                position = source.AbsolutePosition;
            else if ((avatar = m_scene.GetScenePresence(id)) != null) 
                position = avatar.AbsolutePosition;
            else if (ChatTypeEnum.Region == type)
                position = CenterOfRegion;
            else
                return;

            DeliverMessage(type, channel, name, id, msg, position);
        }

        /// <summary>
        /// This method scans over the objects which registered an interest in listen callbacks.
        /// For everyone it finds, it checks if it fits the given filter. If it does,  then
        /// enqueue the message for delivery to the objects listen event handler.
        /// The enqueued ListenerInfo no longer has filter values, but the actually trigged values.
        /// Objects that do an llSay have their messages delivered here and for nearby avatars,
        /// the OnChatFromClient event is used.
        /// </summary>
        /// <param name="type">type of delvery (whisper,say,shout or regionwide)</param>
        /// <param name="channel">channel to sent on</param>
        /// <param name="name">name of sender (object or avatar)</param>
        /// <param name="id">key of sender (object or avatar)</param>
        /// <param name="msg">msg to sent</param>
        public void DeliverMessage(ChatTypeEnum type, int channel, string name, UUID id, string msg, Vector3 position)
        {
            // m_log.DebugFormat("[WorldComm] got[2] type {0}, channel {1}, name {2}, id {3}, msg {4}",
            //                   type, channel, name, id, msg);

            // Determine which listen event filters match the given set of arguments, this results
            // in a limited set of listeners, each belonging a host. If the host is in range, add them
            // to the pending queue.
            foreach (ListenerInfo li in m_listenerManager.GetListeners(UUID.Zero, channel, name, id, msg))
            {
                // Dont process if this message is from yourself!
                if (li.GetHostID().Equals(id))
                    continue;

                SceneObjectPart sPart = m_scene.GetSceneObjectPart(li.GetHostID());
                if (sPart == null)
                    continue;

                double dis =  Util.GetDistanceTo(sPart.AbsolutePosition, position);
                switch (type)
                {
                    case ChatTypeEnum.Whisper:
                        if (dis < m_whisperdistance)
                            QueueMessage(new ListenerInfo(li, name, id, msg));
                        break;

                    case ChatTypeEnum.Say:
                        if (dis < m_saydistance)
                            QueueMessage(new ListenerInfo(li, name, id, msg));
                        break;

                    case ChatTypeEnum.Shout:
                        if (dis < m_shoutdistance)
                            QueueMessage(new ListenerInfo(li, name, id, msg));
                        break;

                    case ChatTypeEnum.Region:
                        QueueMessage(new ListenerInfo(li, name, id, msg));
                        break;
                }
            }
        }

        /// <summary>
        /// Delivers the message to a scene entity.
        /// </summary>
        /// <param name='target'>
        /// Target.
        /// </param>
        /// <param name='channel'>
        /// Channel.
        /// </param>
        /// <param name='name'>
        /// Name.
        /// </param>
        /// <param name='id'>
        /// Identifier.
        /// </param>
        /// <param name='msg'>
        /// Message.
        /// </param>
        public bool DeliverMessageTo(UUID target, int channel, Vector3 pos, string name, UUID id, string msg, out string error)
        {
            error = null;
            // Is id an avatar?
            ScenePresence sp = m_scene.GetScenePresence(target);

            if (sp != null)
            {
                // Send message to avatar
                if (channel == 0)
                {
                    m_scene.SimChatBroadcast(Utils.StringToBytes(msg), ChatTypeEnum.Owner, 0, pos, name, id, false);
                }

                List<SceneObjectGroup> attachments = sp.GetAttachments();

                if (attachments.Count == 0)
                    return true;

                // Get uuid of attachments
                List<UUID> targets = new List<UUID>();
                foreach (SceneObjectGroup sog in attachments)
                {
                    if (!sog.IsDeleted)
                        targets.Add(sog.UUID);
                }

                // Need to check each attachment
                foreach (ListenerInfo li in m_listenerManager.GetListeners(UUID.Zero, channel, name, id, msg))
                {
                    if (li.GetHostID().Equals(id))
                        continue;

                    if (m_scene.GetSceneObjectPart(li.GetHostID()) == null)
                        continue;

                    if (targets.Contains(li.GetHostID()))
                        QueueMessage(new ListenerInfo(li, name, id, msg));
                }

                return true;
            }

            // Need to toss an error here
            if (channel == 0)
            {
                error = "Cannot use llRegionSayTo to message objects on channel 0";
                return false;
            }

            foreach (ListenerInfo li in m_listenerManager.GetListeners(UUID.Zero, channel, name, id, msg))
            {
                // Dont process if this message is from yourself!
                if (li.GetHostID().Equals(id))
                    continue;

                SceneObjectPart sPart = m_scene.GetSceneObjectPart(li.GetHostID());
                if (sPart == null)
                    continue;

                if ( li.GetHostID().Equals(target))
                {
                    QueueMessage(new ListenerInfo(li, name, id, msg));
                    break;
                }
            }

            return true;
        }

        protected void QueueMessage(ListenerInfo li)
        {
            lock (m_pending.SyncRoot)
            {
                m_pending.Enqueue(li);
            }
        }

        /// <summary>
        /// Are there any listen events ready to be dispatched?
        /// </summary>
        /// <returns>boolean indication</returns>
        public bool HasMessages()
        {
            return (m_pending.Count > 0);
        }

        /// <summary>
        /// Pop the first availlable listen event from the queue
        /// </summary>
        /// <returns>ListenerInfo with filter filled in</returns>
        public IWorldCommListenerInfo GetNextMessage()
        {
            ListenerInfo li = null;

            lock (m_pending.SyncRoot)
            {
                li = (ListenerInfo)m_pending.Dequeue();
            }

            return li;
        }

        #endregion

        /********************************************************************
         *
         * Listener Stuff
         *
         * *****************************************************************/

        private void DeliverClientMessage(Object sender, OSChatMessage e)
        {
            if (null != e.Sender)
                DeliverMessage(e.Type, e.Channel, e.Sender.Name, e.Sender.AgentId, e.Message, e.Position);
            else
                DeliverMessage(e.Type, e.Channel, e.From, UUID.Zero, e.Message, e.Position);
        }

        public Object[] GetSerializationData(UUID itemID)
        {
            return m_listenerManager.GetSerializationData(itemID);
        }

        public void CreateFromData(uint localID, UUID itemID, UUID hostID,
                Object[] data)
        {
            m_listenerManager.AddFromData(localID, itemID, hostID, data);
        }
    }

    public class ListenerManager
    {
        private Dictionary<int, List<ListenerInfo>> m_listeners = new Dictionary<int, List<ListenerInfo>>();
        private int m_maxlisteners;
        private int m_maxhandles;
        private int m_curlisteners;

        public ListenerManager(int maxlisteners, int maxhandles)
        {
            m_maxlisteners = maxlisteners;
            m_maxhandles = maxhandles;
            m_curlisteners = 0;
        }

        public int AddListener(uint localID, UUID itemID, UUID hostID, int channel, string name, UUID id, string msg)
        {
            // do we already have a match on this particular filter event?
            List<ListenerInfo> coll = GetListeners(itemID, channel, name, id, msg);

            if (coll.Count > 0)
            {
                // special case, called with same filter settings, return same handle
                // (2008-05-02, tested on 1.21.1 server, still holds)
                return coll[0].GetHandle();
            }

            if (m_curlisteners < m_maxlisteners)
            {
                lock (m_listeners)
                {
                    int newHandle = GetNewHandle(itemID);

                    if (newHandle > 0)
                    {
                        ListenerInfo li = new ListenerInfo(newHandle, localID, itemID, hostID, channel, name, id, msg);

                            List<ListenerInfo> listeners;
                            if (!m_listeners.TryGetValue(channel,out listeners))
                            {
                                listeners = new List<ListenerInfo>();
                                m_listeners.Add(channel, listeners);
                            }
                            listeners.Add(li);
                            m_curlisteners++;

                        return newHandle;
                    }
                }
            }
            return -1;
        }

        public void Remove(UUID itemID, int handle)
        {
            lock (m_listeners)
            {
                foreach (KeyValuePair<int,List<ListenerInfo>> lis in m_listeners)
                {
                    foreach (ListenerInfo li in lis.Value)
                    {
                        if (li.GetItemID().Equals(itemID) && li.GetHandle().Equals(handle))
                        {
                            lis.Value.Remove(li);
                            if (lis.Value.Count == 0)
                            {
                                m_listeners.Remove(lis.Key);
                                m_curlisteners--;
                            }
                            // there should be only one, so we bail out early
                            return;
                        }
                    }
                }
            }
        }

        public void DeleteListener(UUID itemID)
        {
            List<int> emptyChannels = new List<int>();
            List<ListenerInfo> removedListeners = new List<ListenerInfo>();

            lock (m_listeners)
            {
                foreach (KeyValuePair<int,List<ListenerInfo>> lis in m_listeners)
                {
                    foreach (ListenerInfo li in lis.Value)
                    {
                        if (li.GetItemID().Equals(itemID))
                        {
                            // store them first, else the enumerated bails on us
                            removedListeners.Add(li);
                        }
                    }
                    foreach (ListenerInfo li in removedListeners)
                    {
                        lis.Value.Remove(li);
                        m_curlisteners--;
                    }
                    removedListeners.Clear();
                    if (lis.Value.Count == 0)
                    {
                        // again, store first, remove later
                        emptyChannels.Add(lis.Key);
                    }
                }
                foreach (int channel in emptyChannels)
                {
                    m_listeners.Remove(channel);
                }
            }
        }

        public void Activate(UUID itemID, int handle)
        {
            lock (m_listeners)
            {
                foreach (KeyValuePair<int,List<ListenerInfo>> lis in m_listeners)
                {
                    foreach (ListenerInfo li in lis.Value)
                    {
                        if (li.GetItemID().Equals(itemID) && li.GetHandle() == handle)
                        {
                            li.Activate();
                            // only one, bail out
                            return;
                        }
                    }
                }
            }
        }

        public void Dectivate(UUID itemID, int handle)
        {
            lock (m_listeners)
            {
                foreach (KeyValuePair<int,List<ListenerInfo>> lis in m_listeners)
                {
                    foreach (ListenerInfo li in lis.Value)
                    {
                        if (li.GetItemID().Equals(itemID) && li.GetHandle() == handle)
                        {
                            li.Deactivate();
                            // only one, bail out
                            return;
                        }
                    }
                }
            }
        }

        // non-locked access, since its always called in the context of the lock
        private int GetNewHandle(UUID itemID)
        {
            List<int> handles = new List<int>();

            // build a list of used keys for this specific itemID...
            foreach (KeyValuePair<int,List<ListenerInfo>> lis in m_listeners)
            {
                 foreach (ListenerInfo li in lis.Value)
                 {
                     if (li.GetItemID().Equals(itemID))
                         handles.Add(li.GetHandle());
                 }
            }

            // Note: 0 is NOT a valid handle for llListen() to return
            for (int i = 1; i <= m_maxhandles; i++)
            {
                if (!handles.Contains(i))
                    return i;
            }

            return -1;
        }

        // Theres probably a more clever and efficient way to
        // do this, maybe with regex.
        // PM2008: Ha, one could even be smart and define a specialized Enumerator.
        public List<ListenerInfo> GetListeners(UUID itemID, int channel, string name, UUID id, string msg)
        {
            List<ListenerInfo> collection = new List<ListenerInfo>();

            lock (m_listeners)
            {
                List<ListenerInfo> listeners;
                if (!m_listeners.TryGetValue(channel,out listeners))
                {
                    return collection;
                }

                foreach (ListenerInfo li in listeners)
                {
                    if (!li.IsActive())
                    {
                        continue;
                    }
                    if (!itemID.Equals(UUID.Zero) && !li.GetItemID().Equals(itemID))
                    {
                        continue;
                    }
                    if (li.GetName().Length > 0 && !li.GetName().Equals(name))
                    {
                        continue;
                    }
                    if (!li.GetID().Equals(UUID.Zero) && !li.GetID().Equals(id))
                    {
                        continue;
                    }
                    if (li.GetMessage().Length > 0 && !li.GetMessage().Equals(msg))
                    {
                        continue;
                    }
                    collection.Add(li);
                }
            }
            return collection;
        }

        public Object[] GetSerializationData(UUID itemID)
        {
            List<Object> data = new List<Object>();

            lock (m_listeners)
            {
                foreach (List<ListenerInfo> list in m_listeners.Values)
                {
                    foreach (ListenerInfo l in list)
                    {
                        if (l.GetItemID() == itemID)
                            data.AddRange(l.GetSerializationData());
                    }
                }
            }
            return (Object[])data.ToArray();
        }

        public void AddFromData(uint localID, UUID itemID, UUID hostID,
                Object[] data)
        {
            int idx = 0;
            Object[] item = new Object[6];

            while (idx < data.Length)
            {
                Array.Copy(data, idx, item, 0, 6);

                ListenerInfo info =
                        ListenerInfo.FromData(localID, itemID, hostID, item);

                lock (m_listeners)
                {
                    if (!m_listeners.ContainsKey((int)item[2]))
                        m_listeners.Add((int)item[2], new List<ListenerInfo>());
                    m_listeners[(int)item[2]].Add(info);
                }

                idx+=6;
            }
        }
    }

    public class ListenerInfo: IWorldCommListenerInfo
    {
        private bool m_active; // Listener is active or not
        private int m_handle; // Assigned handle of this listener
        private uint m_localID; // Local ID from script engine
        private UUID m_itemID; // ID of the host script engine
        private UUID m_hostID; // ID of the host/scene part
        private int m_channel; // Channel
        private UUID m_id; // ID to filter messages from
        private string m_name; // Object name to filter messages from
        private string m_message; // The message

        public ListenerInfo(int handle, uint localID, UUID ItemID, UUID hostID, int channel, string name, UUID id, string message)
        {
            Initialise(handle, localID, ItemID, hostID, channel, name, id, message);
        }

        public ListenerInfo(ListenerInfo li, string name, UUID id, string message)
        {
            Initialise(li.m_handle, li.m_localID, li.m_itemID, li.m_hostID, li.m_channel, name, id, message);
        }

        private void Initialise(int handle, uint localID, UUID ItemID, UUID hostID, int channel, string name,
                                UUID id, string message)
        {
            m_active = true;
            m_handle = handle;
            m_localID = localID;
            m_itemID = ItemID;
            m_hostID = hostID;
            m_channel = channel;
            m_name = name;
            m_id = id;
            m_message = message;
        }

        public Object[] GetSerializationData()
        {
            Object[] data = new Object[6];

            data[0] = m_active;
            data[1] = m_handle;
            data[2] = m_channel;
            data[3] = m_name;
            data[4] = m_id;
            data[5] = m_message;

            return data;
        }

        public static ListenerInfo FromData(uint localID, UUID ItemID, UUID hostID, Object[] data)
        {
            ListenerInfo linfo = new ListenerInfo((int)data[1], localID,
                    ItemID, hostID, (int)data[2], (string)data[3],
                    (UUID)data[4], (string)data[5]);
            linfo.m_active=(bool)data[0];

            return linfo;
        }

        public UUID GetItemID()
        {
            return m_itemID;
        }

        public UUID GetHostID()
        {
            return m_hostID;
        }

        public int GetChannel()
        {
            return m_channel;
        }

        public uint GetLocalID()
        {
            return m_localID;
        }

        public int GetHandle()
        {
            return m_handle;
        }

        public string GetMessage()
        {
            return m_message;
        }

        public string GetName()
        {
            return m_name;
        }

        public bool IsActive()
        {
            return m_active;
        }

        public void Deactivate()
        {
            m_active = false;
        }

        public void Activate()
        {
            m_active = true;
        }

        public UUID GetID()
        {
            return m_id;
        }
    }
}
