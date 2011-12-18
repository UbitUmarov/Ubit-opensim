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

using System.Collections.Generic;
using System.Reflection;
using log4net;
using Nini.Config;
using OpenSim.Framework;
using OpenMetaverse;

namespace OpenSim.Region.Physics.Manager
{
    public delegate void physicsCrash();

    public delegate void RaycastCallback(bool hitYN, Vector3 collisionPoint, uint localid, float distance, Vector3 normal);
    public delegate void RayCallback(List<ContactResult> list);

    public delegate void JointMoved(PhysicsJoint joint);
    public delegate void JointDeactivated(PhysicsJoint joint);
    public delegate void JointErrorMessage(PhysicsJoint joint, string message); // this refers to an "error message due to a problem", not "amount of joint constraint violation"

    /// <summary>
    /// Contact result from a raycast.
    /// </summary>
    public struct ContactResult
    {
        public Vector3 Pos;
        public float Depth;
        public uint ConsumerID;
        public Vector3 Normal;
    }

    public abstract class PhysicsScene
    {
//        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// Name of this scene.  Useful in debug messages to distinguish one OdeScene instance from another.
        /// </summary>
        public string Name { get; protected set; }

        // The only thing that should register for this event is the SceneGraph
        // Anything else could cause problems.

        public event physicsCrash OnPhysicsCrash;

        public static PhysicsScene Null
        {
            get { return new NullPhysicsScene(); }
        }

        public virtual void TriggerPhysicsBasedRestart()
        {
            physicsCrash handler = OnPhysicsCrash;
            if (handler != null)
            {
                OnPhysicsCrash();
            }
        }

        public abstract void Initialise(IMesher meshmerizer, IConfigSource config,RegionInfo region);

        /// <summary>
        /// Add an avatar
        /// </summary>
        /// <param name="avName"></param>
        /// <param name="position"></param>
        /// <param name="size"></param>
        /// <param name="isFlying"></param>
        /// <returns></returns>
        public abstract PhysicsActor AddAvatar(string avName, Vector3 position, Vector3 size, bool isFlying);

        /// <summary>
        /// Add an avatar
        /// </summary>
        /// <param name="localID"></param>
        /// <param name="avName"></param>
        /// <param name="position"></param>
        /// <param name="size"></param>
        /// <param name="isFlying"></param>
        /// <returns></returns>
        public virtual PhysicsActor AddAvatar(uint localID, string avName, Vector3 position, Vector3 size, bool isFlying)
        {
            PhysicsActor ret = AddAvatar(avName, position, size, isFlying);
            if (ret != null) ret.LocalID = localID;
            return ret;
        }

        /// <summary>
        /// Remove an avatar.
        /// </summary>
        /// <param name="actor"></param>
        public abstract void RemoveAvatar(PhysicsActor actor);

        /// <summary>
        /// Remove a prim.
        /// </summary>
        /// <param name="prim"></param>
        public abstract void RemovePrim(PhysicsActor prim);

        public abstract PhysicsActor AddPrimShape(string primName, PrimitiveBaseShape pbs, Vector3 position,
                                                  Vector3 size, Quaternion rotation, bool isPhysical, uint localid);

        public virtual float TimeDilation
        {
            get { return 1.0f; }
        }

        public virtual bool SupportsNINJAJoints
        {
            get { return false; }
        }

        public virtual PhysicsJoint RequestJointCreation(string objectNameInScene, PhysicsJointType jointType, Vector3 position,
                                            Quaternion rotation, string parms, List<string> bodyNames, string trackedBodyName, Quaternion localRotation)
        { return null; }

        public virtual void RequestJointDeletion(string objectNameInScene)
        { return; }

        public virtual void RemoveAllJointsConnectedToActorThreadLocked(PhysicsActor actor)
        { return; }

        public virtual void DumpJointInfo()
        { return; }

        public event JointMoved OnJointMoved;

        protected virtual void DoJointMoved(PhysicsJoint joint)
        {
            // We need this to allow subclasses (but not other classes) to invoke the event; C# does
            // not allow subclasses to invoke the parent class event.
            if (OnJointMoved != null)
            {
                OnJointMoved(joint);
            }
        }

        public event JointDeactivated OnJointDeactivated;

        protected virtual void DoJointDeactivated(PhysicsJoint joint)
        {
            // We need this to allow subclasses (but not other classes) to invoke the event; C# does
            // not allow subclasses to invoke the parent class event.
            if (OnJointDeactivated != null)
            {
                OnJointDeactivated(joint);
            }
        }

        public event JointErrorMessage OnJointErrorMessage;

        protected virtual void DoJointErrorMessage(PhysicsJoint joint, string message)
        {
            // We need this to allow subclasses (but not other classes) to invoke the event; C# does
            // not allow subclasses to invoke the parent class event.
            if (OnJointErrorMessage != null)
            {
                OnJointErrorMessage(joint, message);
            }
        }

        public virtual Vector3 GetJointAnchor(PhysicsJoint joint)
        { return Vector3.Zero; }

        public virtual Vector3 GetJointAxis(PhysicsJoint joint)
        { return Vector3.Zero; }

        public abstract void AddPhysicsActorTaint(PhysicsActor prim);

        public abstract float Simulate(float timeStep);

        public abstract void GetResults();

        public abstract void SetTerrain(float[] heightMap);

        public abstract void SetWaterLevel(float baseheight);

        public abstract void DeleteTerrain();

        public abstract void Dispose();

        public abstract Dictionary<uint, float> GetTopColliders();

        public abstract bool IsThreaded { get; }

        /// <summary>
        /// True if the physics plugin supports raycasting against the physics scene
        /// </summary>
        public virtual bool SupportsRayCast()
        {
            return false;
        }

        public virtual bool SupportsCombining()
        {
            return false;
        }

        public virtual void CombineTerrain(float[] heightMap, Vector3 pOffsets) { }

//        public virtual void UnCombine(PhysicsScene pScene) {}

        /// <summary>
        /// Queue a raycast against the physics scene.
        /// The provided callback method will be called when the raycast is complete
        /// 
        /// Many physics engines don't support collision testing at the same time as 
        /// manipulating the physics scene, so we queue the request up and callback 
        /// a custom method when the raycast is complete.
        /// This allows physics engines that give an immediate result to callback immediately
        /// and ones that don't, to callback when it gets a result back.
        /// 
        /// ODE for example will not allow you to change the scene while collision testing or
        /// it asserts, 'opteration not valid for locked space'.  This includes adding a ray to the scene.
        /// 
        /// This is named RayCastWorld to not conflict with modrex's Raycast method.
        /// </summary>
        /// <param name="position">Origin of the ray</param>
        /// <param name="direction">Direction of the ray</param>
        /// <param name="length">Length of ray in meters</param>
        /// <param name="retMethod">Method to call when the raycast is complete</param>
        public virtual void RaycastWorld(Vector3 position, Vector3 direction, float length, RaycastCallback retMethod)
        {
            if (retMethod != null)
                retMethod(false, Vector3.Zero, 0, 999999999999f, Vector3.Zero);
        }

        public virtual void RaycastWorld(Vector3 position, Vector3 direction, float length, int Count, RayCallback retMethod)
        {
            if (retMethod != null)
                retMethod(new List<ContactResult>());
        }

        public virtual List<ContactResult> RaycastWorld(Vector3 position, Vector3 direction, float length, int Count)
        {
            return new List<ContactResult>();
        }

        // methods as above but only probing a single actor and not the world
        public virtual void RaycastActor(PhysicsActor actor, Vector3 position, Vector3 direction, float length, RaycastCallback retMethod)
        {
            if (retMethod != null)
                retMethod(false, Vector3.Zero, 0, 999999999999f, Vector3.Zero);
        }

        public virtual void RaycastActor(PhysicsActor actor, Vector3 position, Vector3 direction, float length, int Count, RayCallback retMethod)
        {
            if (retMethod != null)
                retMethod(new List<ContactResult>());
        }

        public virtual List<ContactResult> RaycastActor(PhysicsActor actor, Vector3 position, Vector3 direction, float length, int Count)
        {
            return new List<ContactResult>();
        }

    }
}
