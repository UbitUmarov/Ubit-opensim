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
using System.Reflection;

using OpenSim.Framework;
using OpenSim.Framework.Client;
using log4net;

namespace OpenSim.Framework
{
    public class PriorityQueue
    {
//        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public delegate bool UpdatePriorityHandler(ref uint priority, ISceneEntity entity);


        private MinHeap<MinHeapItem> m_heap;
        private Dictionary<uint, LookupItem> m_lookupTable;

        /// <summary>
        /// Lock for enqueue and dequeue operations on the priority queue
        /// </summary>
        private object m_syncRoot = new object();
        public object SyncRoot {
            get { return this.m_syncRoot; }
        }

#region constructor
        public PriorityQueue() : this(MinHeap<MinHeapItem>.DEFAULT_CAPACITY) { }

        public PriorityQueue(int capacity)
        {
            m_lookupTable = new Dictionary<uint, LookupItem>(capacity);
            m_heap = new MinHeap<MinHeapItem>(capacity);

        }
#endregion Constructor

#region PublicMethods
        /// <summary>
        /// Return the number of items in the queues
        /// </summary>
        public int Count
        {
            get
            {
            return m_heap.Count;
 
            }
        }

        /// <summary>
        /// Enqueue an item into the specified priority queue
        /// </summary>
        public bool Enqueue(uint priority, IEntityUpdate value)
        {
            LookupItem lookup;

            uint localid = value.Entity.LocalId;

            int age = -5; //delay start of promotion of stuked updates  to tune up ...
            if (m_lookupTable.TryGetValue(localid, out lookup))
            {
                age = m_heap[lookup.Handle].Age;
                value.Update(m_heap[lookup.Handle].Value);
                m_heap.Remove(lookup.Handle);
            }

            m_heap.Add(new MinHeapItem(age, priority, value), ref lookup.Handle);
            m_lookupTable[localid] = lookup;

            return true;
        }

        /// <summary>
        /// Remove an item from one of the queues. Specifically, it removes the
        /// oldest item from the next queue in order to provide fair access to
        /// all of the queues
        /// </summary>
        public bool TryDequeue(out IEntityUpdate value, out Int32 timeinqueue)
            {
            if (m_heap.Count > 0)
                {
                MinHeapItem item = m_heap.RemoveMin();
                m_lookupTable.Remove(item.Value.Entity.LocalId);
                timeinqueue = Util.EnvironmentTickCountSubtract(item.EntryTime);
                value = item.Value;
                return true;
                }
            timeinqueue = 0;
            value = default(IEntityUpdate);
            return false;
            }

        /// <summary>
        /// Reapply the prioritization function to each of the updates currently
        /// stored in the priority queues. 
        /// </summary
        public void Reprioritize(UpdatePriorityHandler handler)
        {
            MinHeapItem item;
            foreach (LookupItem lookup in new List<LookupItem>(this.m_lookupTable.Values))
            {
                if (m_heap.TryGetValue(lookup.Handle, out item))
                {
                    uint nprio = 0;
                    uint localid = item.Value.Entity.LocalId;

                    if (handler(ref nprio, item.Value.Entity))
                        {
                        /*
                        int age = item.Age;

                        if(age <1)
                            {
                            age++;
                            }
                        else
                            {
                            age = age << 1;
                            nprio = nprio >> age;
                            }
                        item.Age=age;
                        */
                        if (item.Priority != nprio)
                            {
                            item.Priority = nprio;

                            m_heap.Remove(lookup.Handle);
                            LookupItem litem = lookup;
                            m_heap.Add(new MinHeapItem(item), ref litem.Handle);
                            m_lookupTable[localid] = litem;
                            }
                        }
                    else
                    {
                        // m_log.WarnFormat("[PQUEUE]: UpdatePriorityHandler returned false for {0}",item.Value.Entity.UUID);
                        m_heap.Remove(lookup.Handle);
                        this.m_lookupTable.Remove(localid);
                    }
                }
            }
        }

        /// <summary>
        /// </summary>
        public override string ToString()
        {
            string s = "";
            s += String.Format("{0,7} ",m_heap.Count);
            return s;
        }

#endregion PublicMethods

#region MinHeapItem
        private struct MinHeapItem : IComparable<MinHeapItem>
        {
            private IEntityUpdate value;
            internal IEntityUpdate Value {
                get {
                    return this.value;
                }
            }

            private Int32 entrytime;
            internal Int32 EntryTime {
                get {
                    return this.entrytime;
                }
            }

            private int age;
            internal int Age
                {
                get
                    {
                    return this.age;
                    }
                set
                    {
                    age = value;
                    }
                }
            private uint priority;
            internal uint Priority
                {
                get
                    {
                    return this.priority;
                    }
                set
                    {
                    priority = value;
                    }
                    
                }

            internal MinHeapItem(MinHeapItem other)
            {
                this.entrytime = other.entrytime;
                this.age = other.age;
                this.value = other.value;
                this.priority = other.priority;
            }

            internal MinHeapItem(int age, uint priority, IEntityUpdate value)
            {
                this.entrytime = Util.EnvironmentTickCount();
                this.age = age;
                this.value = value;
                this.priority = priority;
            }

            public override string ToString()
            {
            return String.Format("[{0},{1},{2}", age, priority, value.Entity.LocalId);
            }

            public int CompareTo(MinHeapItem other)
            {
                // I'm assuming that the root part of an SOG is added to the update queue
                // before the component parts
            return Comparer<UInt64>.Default.Compare(this.Priority, other.Priority);
            }
        }
#endregion

#region LookupItem
        private struct LookupItem
        {
            internal IHandle Handle;
        }
#endregion
    }
}
