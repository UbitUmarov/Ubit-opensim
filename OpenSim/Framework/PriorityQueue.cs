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

        /// <summary>
        /// Total number of queues (priorities) available
        /// </summary>
        public const uint NumberOfQueues = 12;
        /// <summary>
        /// Number of queuest (priorities) that are processed immediately
        /// </summary.
        public const uint NumberOfImmediateQueues = 2;

        private MinHeap<MinHeapItem>[] m_heaps = new MinHeap<MinHeapItem>[NumberOfQueues];
        private Dictionary<uint, LookupItem> m_lookupTable;

        // next request is a counter of the number of updates queued, it provides
        // a total ordering on the updates coming through the queue and is more
        // lightweight (and more discriminating) than tick count
        private UInt64 m_nextRequest = 0;

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
            for (int i = 0; i < m_heaps.Length; ++i)
                m_heaps[i] = new MinHeap<MinHeapItem>(capacity);
        }
#endregion Constructor

// Ubit: maps priority into queue 
// hack to keep compatibility with previus: priority of 0 and 1 directly select imediate queues
// if priority is distance square then map is aprox:
//      0           1             2     3      4     5      6      7     8     9    10    11
// imediateQueue imediateQueue  <32m  <45m   <64m  <90m   <128m  <181m <256m <362 <512m TheRest
// must be changed if number of queues change !!
        private uint GetQueue(uint priority)
            {

            if (priority < 2) // the hack
                return priority;

            uint pqueue;

            if (priority < 1024.0)
                pqueue = NumberOfImmediateQueues;
            else
                {
                float tmp = (float)Math.Log((double)priority) * 1.4426950408889634073599246810019f - 7f;
                // for a map identical to original:
                // 1st constant is 1/(2*log(2)) (natural log): 0.72134752044448170367996234050095f
                // 2st constant is log(10)/ln(2) + 1 : 4.3219280948873623478703194294894f
                // now 
                // 1st constant is 1/(log(2)) (natural log)
                // 2st constant shift into right one: 


                pqueue = (uint)tmp + NumberOfImmediateQueues;
                if (pqueue >= NumberOfQueues)
                    {
                    // Ooops...
                    pqueue = PriorityQueue.NumberOfQueues - 1;
                    }
                }
            return pqueue;
            }

#region PublicMethods


        /// <summary>
        /// Return the number of items in the queues
        /// </summary>
        public int Count
        {
            get
            {
            int count = 0;
            for (int i = 0; i < m_heaps.Length; ++i)
                count += m_heaps[i].Count;

            return count; 
            }
        }

        /// <summary>
        /// Enqueue an item into the specified priority queue
        /// </summary>
        public bool Enqueue(uint priority, IEntityUpdate value)
        {
            LookupItem lookup;

            uint localid = value.Entity.LocalId;

            uint pqueue = GetQueue(priority);

            UInt64 entry = m_nextRequest++;
            if (m_lookupTable.TryGetValue(localid, out lookup))
                {
                entry = lookup.Heap[lookup.Handle].EntryOrder;
                value.Update(lookup.Heap[lookup.Handle].Value);
                lookup.Heap.Remove(lookup.Handle);
                }

            pqueue = Util.Clamp<uint>(pqueue, 0, NumberOfQueues - 1);
            lookup.Heap = m_heaps[pqueue];
            lookup.Heap.Add(new MinHeapItem(pqueue, entry, priority, value), ref lookup.Handle);
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
            // get a next priority item
            for (int iq = 0; iq < NumberOfQueues; iq++)
                {
                if (m_heaps[iq].Count > 0)
                    {
                    MinHeapItem item = m_heaps[iq].RemoveMin();
                    m_lookupTable.Remove(item.Value.Entity.LocalId);
                    timeinqueue = Util.EnvironmentTickCountSubtract(item.EntryTime);
                    value = item.Value;

                    return true;
                    }
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
            if (lookup.Heap.TryGetValue(lookup.Handle, out item))
                {
                uint pqueue;
                uint prio = item.Priority;
                uint localid = item.Value.Entity.LocalId;
                uint nprio = prio;

                if (handler(ref nprio, item.Value.Entity))
                    {
                    // unless the priority queue has changed, there is no need to modify
                    // the entry
                    if (nprio != prio)
                        {
                        lookup.Heap.Remove(lookup.Handle);
                        LookupItem litem = lookup;
                        pqueue = GetQueue(nprio);
                        litem.Heap = m_heaps[pqueue];
                        litem.Heap.Add(new MinHeapItem(pqueue, item), ref litem.Handle);
                        m_lookupTable[localid] = litem;
                        }
                    }
                else
                    {
                    // m_log.WarnFormat("[PQUEUE]: UpdatePriorityHandler returned false for {0}",item.Value.Entity.UUID);
                    lookup.Heap.Remove(lookup.Handle);
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
            for (int i = 0; i < NumberOfQueues; i++)
                s += String.Format("{0,7} ", m_heaps[i].Count);
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

            private uint pqueue;
            internal uint PriorityQueue {
                get {
                    return this.pqueue;
                }
            }

            private Int32 entrytime;
            internal Int32 EntryTime {
                get {
                    return this.entrytime;
                }
            }

            private UInt64 entryorder;
            internal UInt64 EntryOrder
                {
                get
                    {
                    return this.entryorder;
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

            internal MinHeapItem(uint pqueue, MinHeapItem other)
                {
                this.entrytime = other.entrytime;
                this.entryorder = other.entryorder;
                this.value = other.value;
                this.priority = other.priority;
                this.pqueue = pqueue;

                }

            internal MinHeapItem(uint pqueue, UInt64 entryorder, uint priority, IEntityUpdate value)
                {
                this.entrytime = Util.EnvironmentTickCount();
                this.entryorder = entryorder;
                this.value = value;
                this.priority = priority;
                this.pqueue = pqueue;
                }

            public override string ToString()
            {
            return String.Format("[{0},{1},{2},{3}]", pqueue, entryorder, priority, value.Entity.LocalId);
            }

            public int CompareTo(MinHeapItem other)
            {
                // I'm assuming that the root part of an SOG is added to the update queue
                // before the component parts
            return Comparer<UInt64>.Default.Compare(this.EntryOrder, other.EntryOrder);
            }
        }
#endregion

#region LookupItem
        private struct LookupItem
        {
            internal MinHeap<MinHeapItem> Heap;
            internal IHandle Handle;
        }
#endregion
    }
}
