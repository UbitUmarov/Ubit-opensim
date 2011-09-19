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
using log4net;
using Nini.Config;
using OpenSim.Framework;
using OpenMetaverse;
using OpenSim.Region.Physics.Manager;

/*
 * Steps to add a new prioritization policy:
 * 
 *  - Add a new value to the UpdatePrioritizationSchemes enum.
 *  - Specify this new value in the [InterestManagement] section of your
 *    OpenSim.ini. The name in the config file must match the enum value name
 *    (although it is not case sensitive).
 *  - Write a new GetPriorityBy*() method in this class.
 *  - Add a new entry to the switch statement in GetUpdatePriority() that calls
 *    your method.
 */

namespace OpenSim.Region.Framework.Scenes
{
    public enum UpdatePrioritizationSchemes
    {
        Time = 0,
        Distance = 1,
        SimpleAngularDistance = 2,
        FrontBack = 3,
        BestAvatarResponsiveness = 4,
    }

    public class Prioritizer
    {
        private static readonly ILog m_log = LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        
        private Scene m_scene;

        public Prioritizer(Scene scene)
        {
            m_scene = scene;
        }

        /// <summary>
        /// Returns the priority for a update
        /// </summary>
        public uint GetUpdatePriority(IClientAPI client, ISceneEntity entity)
        {
            // If entity is null we have a serious problem
            if (entity == null)
            {
                m_log.WarnFormat("[PRIORITIZER] attempt to prioritize null entity");
                throw new InvalidOperationException("Prioritization entity not defined");
            }

            // If this is an update for our own avatar give it the highest priority
            if (client.AgentId == entity.UUID)
                return 0;

            uint priority;
            
            switch (m_scene.UpdatePrioritizationScheme)
            {
                case UpdatePrioritizationSchemes.Time:
                    priority = GetPriorityByTime(client, entity);
                    break;
                case UpdatePrioritizationSchemes.Distance:
                    priority = GetPriorityByDistance(client, entity);
                    break;
                case UpdatePrioritizationSchemes.SimpleAngularDistance:
                    priority = GetPriorityByDistance(client, entity); // TODO: Reimplement SimpleAngularDistance
                    break;
                case UpdatePrioritizationSchemes.FrontBack:
                    priority = GetPriorityByFrontBack(client, entity);
                    break;
                case UpdatePrioritizationSchemes.BestAvatarResponsiveness:
                    priority = GetPriorityByBestAvatarResponsiveness(client, entity);
                    break;
                default:
                    throw new InvalidOperationException("UpdatePrioritizationScheme not defined.");
            }
            
            return priority;
        }

        private uint GetPriorityByTime(IClientAPI client, ISceneEntity entity)
        {
            // And anything attached to this avatar gets top priority as well
            if (entity is SceneObjectPart)
            {
                SceneObjectPart sop = (SceneObjectPart)entity;
                if (sop.ParentGroup.IsAttachment && client.AgentId == sop.ParentGroup.AttachedAvatar)
                    return 1;
            }

        //            return PriorityQueue.NumberOfImmediateQueues; // first queue past the immediate queues
            return 2; // first queue past the immediate queues
        }

        private uint GetPriorityByDistance(IClientAPI client, ISceneEntity entity)
        {
            // And anything attached to this avatar gets top priority as well
            if (entity is SceneObjectPart)
            {
                SceneObjectPart sop = (SceneObjectPart)entity;
                if (sop.ParentGroup.IsAttachment && client.AgentId == sop.ParentGroup.AttachedAvatar)
                    return 1;
            }

            return ComputeDistancePriority(client,entity,false);
        }
        
        private uint GetPriorityByFrontBack(IClientAPI client, ISceneEntity entity)
        {
            // And anything attached to this avatar gets top priority as well
            if (entity is SceneObjectPart)
            {
                SceneObjectPart sop = (SceneObjectPart)entity;
                if (sop.ParentGroup.IsAttachment && client.AgentId == sop.ParentGroup.AttachedAvatar)
                    return 1;
            }

            return ComputeDistancePriority(client,entity,true);
        }

        private uint GetPriorityByBestAvatarResponsiveness(IClientAPI client, ISceneEntity entity)
        {
            uint pqueue = ComputeDistancePriority(client,entity,true);

            ScenePresence presence = m_scene.GetScenePresence(client.AgentId);
            if (presence != null)
            {
                if (!presence.IsChildAgent)
                {
                    // All avatars other than our own go into pqueue 1
                    if (entity is ScenePresence)
                        return 1;
                    
                    if (entity is SceneObjectPart)
                    {
                        // Attachments are high priority, 
                        if (((SceneObjectPart)entity).ParentGroup.IsAttachment)
                            return 1;

                        // Non physical prims are lower priority than physical prims
                        PhysicsActor physActor = ((SceneObjectPart)entity).ParentGroup.RootPart.PhysActor;
                        if (physActor == null || !physActor.IsPhysical)
                            pqueue++;
                    }
                }
            }

            return pqueue;
        }

        private uint ComputeDistancePriority(IClientAPI client, ISceneEntity entity, bool useFrontBack)
        {
            // Get this agent's position
            ScenePresence presence = m_scene.GetScenePresence(client.AgentId);
            if (presence == null)
            {
                // this shouldn't happen, it basically means that we are prioritizing 
                // updates to send to a client that doesn't have a presence in the scene
                // seems like there's race condition here...

                // m_log.WarnFormat("[PRIORITIZER] attempt to use agent {0} not in the scene",client.AgentId);
                // throw new InvalidOperationException("Prioritization agent not defined");
//                return PriorityQueue.NumberOfQueues - 1;
            return uint.MaxValue-2;
            }
                
            Vector3 entityPos;
            float distancesq;

            // Use the camera position for local agents and avatar position for remote agents
//            Vector3 presencePos = (presence.IsChildAgent) ?
//                presence.AbsolutePosition :
//                presence.CameraPosition;

            // Ubit..  crap use both if 2 distante
            Vector3 presencePos = presence.AbsolutePosition;
            Vector3 presenceCPos = presence.CameraPosition;  // crap C# 
            float Cdistancesq =0;
            bool checkcam = false;
            if (!presence.IsChildAgent)
                {
//              presenceCPos = presence.CameraPosition;
                if (Vector3.DistanceSquared(presencePos, presenceCPos) > 256) // 16m apart
                    checkcam = true;
                }
            

            if (entity is SceneObjectGroup)
                {
                SceneObjectGroup group = (entity as SceneObjectGroup);
                entityPos = group.AbsolutePosition + group.OOBoffset * group.GroupRotation;
                distancesq = Vector3.DistanceSquared(presencePos, entityPos) - group.BSphereRadiusSQ;
                if (checkcam)
                    Cdistancesq = Vector3.DistanceSquared(presenceCPos, entityPos) - group.BSphereRadiusSQ;
                }
            else if (entity is SceneObjectPart)
                {
                SceneObjectPart p = (SceneObjectPart)entity;
                if ((entity as SceneObjectPart).ParentID == 0)
                    {
                    SceneObjectGroup group = p.ParentGroup;
                    entityPos = group.AbsolutePosition + group.OOBoffset * group.GroupRotation;
                    distancesq = Vector3.DistanceSquared(presencePos, entityPos) - group.BSphereRadiusSQ;
                    if (checkcam)
                        Cdistancesq = Vector3.DistanceSquared(presenceCPos, entityPos) - group.BSphereRadiusSQ;
                    }
                else
                    {
                    entityPos = p.GetWorldPosition() + p.OOBoffset * p.GetWorldRotation();
                    distancesq = p.clampedAABdistanceToSQ(presencePos) + 1.0f;
                    if (checkcam)
                        Cdistancesq = p.clampedAABdistanceToSQ(presenceCPos) + 1.0f;
                    }
                }
            else
                {
                entityPos = entity.AbsolutePosition;
                distancesq = Vector3.DistanceSquared(presencePos, entityPos);
                if (checkcam)
                    Cdistancesq = Vector3.DistanceSquared(presenceCPos, entityPos);;
                }

            if (checkcam) // if using cam use smaller distance
                distancesq = (distancesq < Cdistancesq ? distancesq : Cdistancesq);

            if (distancesq < 0)
                distancesq = 0;

            uint prio = (uint) distancesq + 2; // (+2 fake old imediate queues)

            // If this is a root agent, then determine front & back
            // Bump up the priority queue (drop the priority) for any objects behind the avatar
            
            if (useFrontBack && ! presence.IsChildAgent)
            {
                // Root agent, decrease priority for objects behind us
                Vector3 camPosition = presence.CameraPosition;
                Vector3 camAtAxis = presence.CameraAtAxis;

                // Plane equation
                float d = -Vector3.Dot(camPosition, camAtAxis);
                float p = Vector3.Dot(camAtAxis, entityPos) + d;
                if (p < 0.0f) 
                    prio++;
            }

            return prio;
        }
    }
}
