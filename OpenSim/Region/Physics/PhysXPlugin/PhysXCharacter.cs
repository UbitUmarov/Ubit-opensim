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
using Nini.Config;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;
using PhysXWrapper;
using Quaternion=OpenMetaverse.Quaternion;
using System.Reflection;
using log4net;
using OpenMetaverse;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    public class PhysXCharacter : PhysicsActor
    {
        private Vector3 _position;
        private Vector3 _velocity;
        private Vector3 m_rotationalVelocity = Vector3.Zero;
        private Vector3 _acceleration;
        private NxCharacter _character;
        private bool flying;
        private bool iscolliding = false;
        private float gravityAccel;

        public PhysXCharacter(NxCharacter character)
        {
            _character = character;
        }

        public override int PhysicsActorType
        {
            get { return (int) ActorTypes.Agent; }
            set { return; }
        }

        public override bool Building { get; set; }

        public override bool SetAlwaysRun
        {
            get { return false; }
            set { return; }
        }

        public override uint LocalID
        {
            set { return; }
        }

        public override bool Grabbed
        {
            set { return; }
        }

        public override bool Selected
        {
            set { return; }
        }

        public override float Buoyancy
        {
            get { return 0f; }
            set { return; }
        }

        public override bool FloatOnWater
        {
            set { return; }
        }

        public override bool IsPhysical
        {
            get { return false; }
            set { return; }
        }

        public override bool ThrottleUpdates
        {
            get { return false; }
            set { return; }
        }

        public override bool Flying
        {
            get { return flying; }
            set { flying = value; }
        }

        public override bool IsColliding
        {
            get { return iscolliding; }
            set { iscolliding = value; }
        }

        public override bool CollidingGround
        {
            get { return false; }
            set { return; }
        }

        public override bool CollidingObj
        {
            get { return false; }
            set { return; }
        }

        public override Vector3 RotationalVelocity
        {
            get { return m_rotationalVelocity; }
            set { m_rotationalVelocity = value; }
        }

        public override bool Stopped
        {
            get { return false; }
        }

        public override Vector3 Position
        {
            get { return _position; }
            set
            {
                _position = value;
                Vec3 ps = new Vec3();
                ps.X = value.X;
                ps.Y = value.Y;
                ps.Z = value.Z;
                _character.Position = ps;
            }
        }

        public override Vector3 Size
        {
            get { return Vector3.Zero; }
            set { }
        }

        public override float Mass
        {
            get { return 0f; }
        }

        public override Vector3 Force
        {
            get { return Vector3.Zero; }
            set { return; }
        }

        public override int VehicleType
        {
            get { return 0; }
            set { return; }
        }

        public override void VehicleFloatParam(int param, float value)
        {
        }

        public override void VehicleVectorParam(int param, Vector3 value)
        {
        }

        public override void VehicleRotationParam(int param, Quaternion rotation)
        {
        }

        public override void VehicleFlags(int param, bool remove)
        {
        }

        public override void SetVolumeDetect(int param)
        {
        }

        public override Vector3 CenterOfMass
        {
            get { return Vector3.Zero; }
        }

        public override Vector3 GeometricCenter
        {
            get { return Vector3.Zero; }
        }

        public override Vector3 Velocity
        {
            get { return _velocity; }
            set { _velocity = value; }
        }

        public override float CollisionScore
        {
            get { return 0f; }
            set { }
        }

        public override bool Kinematic
        {
            get { return false; }
            set { }
        }

        public override Quaternion Orientation
        {
            get { return Quaternion.Identity; }
            set { }
        }

        public override Vector3 Acceleration
        {
            get { return _acceleration; }
        }

        public void SetAcceleration(Vector3 accel)
        {
            _acceleration = accel;
        }

        public override void AddForce(Vector3 force, bool pushforce)
        {
        }

        public override Vector3 Torque
        {
            get { return Vector3.Zero; }
            set { return; }
        }

        public override void AddAngularForce(Vector3 force, bool pushforce)
        {
        }

        public override void link(PhysicsActor obj)
        {
        }

        public override void delink()
        {
        }

        public override void LockAngularMotion(Vector3 axis)
        {
        }

        public override void SetMomentum(Vector3 momentum)
        {
        }

        public void Move(float timeStep)
        {
            Vec3 vec = new Vec3();
            vec.X = _velocity.X*timeStep;
            vec.Y = _velocity.Y*timeStep;
            if (flying)
            {
                vec.Z = (_velocity.Z)*timeStep;
            }
            else
            {
                gravityAccel += -9.8f;
                vec.Z = (gravityAccel + _velocity.Z)*timeStep;
            }
            int res = _character.Move(vec);
            if (res == 1)
            {
                gravityAccel = 0;
            }
        }

        public override PrimitiveBaseShape Shape
        {
            set { return; }
        }

        public void UpdatePosition()
        {
            Vec3 vec = _character.Position;
            _position.X = vec.X;
            _position.Y = vec.Y;
            _position.Z = vec.Z;
        }

        public override void CrossingFailure()
        {
        }

        public override Vector3 PIDTarget { set { return; } }
        public override bool PIDActive { set { return; } }
        public override float PIDTau { set { return; } }

        public override float PIDHoverHeight { set { return; } }
        public override bool PIDHoverActive { set { return; } }
        public override PIDHoverType PIDHoverType { set { return; } }
        public override float PIDHoverTau { set { return; } }

        public override Quaternion APIDTarget
        {
            set { return; }
        }

        public override bool APIDActive
        {
            set { return; }
        }

        public override float APIDStrength
        {
            set { return; }
        }

        public override float APIDDamping
        {
            set { return; }
        }

        public override void SubscribeEvents(int ms)
        {

        }
        public override void UnSubscribeEvents()
        {

        }
        public override bool SubscribedEvents()
        {
            return false;
        }
    }
}
