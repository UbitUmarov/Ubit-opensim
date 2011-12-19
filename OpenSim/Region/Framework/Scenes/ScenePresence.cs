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
using System.Xml;
using System.Collections.Generic;
using System.Reflection;
using System.Timers;
using OpenMetaverse;
using log4net;
using OpenSim.Framework;
using OpenSim.Framework.Client;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes.Animation;
using OpenSim.Region.Framework.Scenes.Types;
using OpenSim.Region.Physics.Manager;
using GridRegion = OpenSim.Services.Interfaces.GridRegion;
using OpenSim.Services.Interfaces;

namespace OpenSim.Region.Framework.Scenes
{
    enum ScriptControlled : uint
    {
        CONTROL_ZERO = 0,
        CONTROL_FWD = 1,
        CONTROL_BACK = 2,
        CONTROL_LEFT = 4,
        CONTROL_RIGHT = 8,
        CONTROL_UP = 16,
        CONTROL_DOWN = 32,
        CONTROL_ROT_LEFT = 256,
        CONTROL_ROT_RIGHT = 512,
        CONTROL_LBUTTON = 268435456,
        CONTROL_ML_LBUTTON = 1073741824
    }

    struct ScriptControllers
    {
        public UUID itemID;
        public ScriptControlled ignoreControls;
        public ScriptControlled eventControls;
    }

    public delegate void SendCourseLocationsMethod(UUID scene, ScenePresence presence, List<Vector3> coarseLocations, List<UUID> avatarUUIDs);

    public class ScenePresence : EntityBase, IScenePresence
    {
//        ~ScenePresence()
//        {
//            m_log.Debug("[SCENE PRESENCE] Destructor called");
//        }

        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public PresenceType PresenceType { get; private set; }

//        private static readonly byte[] DEFAULT_TEXTURE = AvatarAppearance.GetDefaultTexture().GetBytes();
        private static readonly Array DIR_CONTROL_FLAGS = Enum.GetValues(typeof(Dir_ControlFlags));
        private static readonly Vector3 HEAD_ADJUSTMENT = new Vector3(0f, 0f, 0.3f);
        
        /// <summary>
        /// Experimentally determined "fudge factor" to make sit-target positions
        /// the same as in SecondLife. Fudge factor was tested for 36 different
        /// test cases including prims of type box, sphere, cylinder, and torus,
        /// with varying parameters for sit target location, prim size, prim
        /// rotation, prim cut, prim twist, prim taper, and prim shear. See mantis
        /// issue #1716
        /// </summary>
        public static readonly Vector3 SIT_TARGET_ADJUSTMENT = new Vector3(0.0f, 0.0f, 0.35f);
        public static readonly Vector3 OLD_SIT_TARGET_ADJUSTMENT = new Vector3(0.1f, 0.0f, 0.3f);

        /// <summary>
        /// Movement updates for agents in neighboring regions are sent directly to clients.
        /// This value only affects how often agent positions are sent to neighbor regions
        /// for things such as distance-based update prioritization
        /// </summary>
        public static readonly float SIGNIFICANT_MOVEMENT = 2.0f;

        public UUID currentParcelUUID = UUID.Zero;

        protected ScenePresenceAnimator m_animator;
        /// <value>
        /// The animator for this avatar
        /// </value>
        public ScenePresenceAnimator Animator
        {
            get { return m_animator; }
            private set { m_animator = value; }
        }

        /// <summary>
        /// Attachments recorded on this avatar.
        /// </summary>
        /// <remarks>
        /// TODO: For some reason, we effectively have a list both here and in Appearance.  Need to work out if this is
        /// necessary.
        /// </remarks>
        protected List<SceneObjectGroup> m_attachments = new List<SceneObjectGroup>();

        public Object AttachmentsSyncLock { get; private set; }

        private Dictionary<UUID, ScriptControllers> scriptedcontrols = new Dictionary<UUID, ScriptControllers>();
        private ScriptControlled IgnoredControls = ScriptControlled.CONTROL_ZERO;
        private ScriptControlled LastCommands = ScriptControlled.CONTROL_ZERO;
        private bool MouseDown = false;
//        private SceneObjectGroup proxyObjectGroup;
        //private SceneObjectPart proxyObjectPart = null;
        public Vector3 lastKnownAllowedPosition;
        public bool sentMessageAboutRestrictedParcelFlyingDown;
        public Vector4 CollisionPlane = Vector4.UnitW;

        private Vector3 m_lastPosition;
        private Quaternion m_lastRotation;
        private Vector3 m_lastVelocity;

        private Vector3? m_forceToApply;
        private int m_userFlags;
        public int UserFlags
        {
            get { return m_userFlags; }
        }
        private bool m_flyingOld;		// add for fly velocity control
        public bool WasFlying
        {
            get { return m_wasFlying; }
        }
        private bool m_wasFlying;		// add for fly velocity control

//        private int m_lastColCount = -1;		//KF: Look for Collision chnages
//        private int m_updateCount = 0;			//KF: Update Anims for a while
//        private static readonly int UPDATE_COUNT = 10;		// how many frames to update for

        private TeleportFlags m_teleportFlags;
        public TeleportFlags TeleportFlags
        {
            get { return m_teleportFlags; }
            set { m_teleportFlags = value; }
        }

        private uint m_requestedSitTargetID;
        private UUID m_requestedSitTargetUUID;
        private Vector3 m_requestedSitOffset = new Vector3();
        private Quaternion m_requestedSitOrientation = new Quaternion();

        /// <summary>
        /// Are we sitting on the ground?
        /// </summary>
        public bool SitGround { get; private set; }

        private SendCourseLocationsMethod m_sendCourseLocationsMethod;

        //private Vector3 m_requestedSitOffset = new Vector3();

        private Vector3 m_LastFinitePos;

        private float m_sitAvatarHeight = 2.0f;

        private Vector3 m_lastChildAgentUpdatePosition;
        private Vector3 m_lastChildAgentUpdateCamPosition;

        private const int LAND_VELOCITYMAG_MAX = 12;

        public bool IsRestrictedToRegion;

        public string JID = String.Empty;

        private float m_health = 100f;

        protected ulong crossingFromRegion;

        private readonly Vector3[] Dir_Vectors = new Vector3[11];

        protected Timer m_reprioritization_timer;
        protected bool m_reprioritizing;
        protected bool m_reprioritization_called;

        private Quaternion m_headrotation = Quaternion.Identity;

        private string m_nextSitAnimation = String.Empty;

        //PauPaw:Proper PID Controler for autopilot************
        public bool MovingToTarget { get; private set; }
        public Vector3 MoveToPositionTarget { get; private set; }

        /// <summary>
        /// Controls whether an avatar automatically moving to a target will land when it gets there (if flying).
        /// </summary>
        public bool LandAtTarget { get; private set; }

        private bool m_followCamAuto;

        private int m_movementUpdateCount;
        private const int NumMovementsBetweenRayCast = 5;

        private bool CameraConstraintActive;
        //private int m_moveToPositionStateStatus;
        //*****************************************************

        protected AvatarAppearance m_appearance;

        public AvatarAppearance Appearance
        {
            get { return m_appearance; }
            set
            {
                m_appearance = value;
//                m_log.DebugFormat("[SCENE PRESENCE]: Set appearance for {0} to {1}", Name, value);
            }
        }

        /// <summary>
        /// Copy of the script states while the agent is in transit. This state may
        /// need to be placed back in case of transfer fail.
        /// </summary>
        public List<string> InTransitScriptStates
        {
            get { return m_InTransitScriptStates; }
            private set { m_InTransitScriptStates = value; }
        }
        private List<string> m_InTransitScriptStates = new List<string>();

        /// <summary>
        /// Implemented Control Flags
        /// </summary>
        private enum Dir_ControlFlags
        {
            DIR_CONTROL_FLAG_FORWARD = AgentManager.ControlFlags.AGENT_CONTROL_AT_POS,
            DIR_CONTROL_FLAG_BACK = AgentManager.ControlFlags.AGENT_CONTROL_AT_NEG,
            DIR_CONTROL_FLAG_LEFT = AgentManager.ControlFlags.AGENT_CONTROL_LEFT_POS,
            DIR_CONTROL_FLAG_RIGHT = AgentManager.ControlFlags.AGENT_CONTROL_LEFT_NEG,
            DIR_CONTROL_FLAG_UP = AgentManager.ControlFlags.AGENT_CONTROL_UP_POS,
            DIR_CONTROL_FLAG_DOWN = AgentManager.ControlFlags.AGENT_CONTROL_UP_NEG,
            DIR_CONTROL_FLAG_FORWARD_NUDGE = AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_AT_POS,
            DIR_CONTROL_FLAG_BACKWARD_NUDGE = AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_AT_NEG,
            DIR_CONTROL_FLAG_LEFT_NUDGE = AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_LEFT_POS,
            DIR_CONTROL_FLAG_RIGHT_NUDGE = AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_LEFT_NEG,
            DIR_CONTROL_FLAG_DOWN_NUDGE = AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_NEG
        }
        
        /// <summary>
        /// Position at which a significant movement was made
        /// </summary>
        private Vector3 posLastSignificantMove;

        // For teleports and crossings callbacks
        string m_callbackURI;
        UUID m_originRegionID;

        /// <value>
        /// Script engines present in the scene
        /// </value>
        private IScriptModule[] m_scriptEngines;

        #region Properties

        protected PhysicsActor m_physicsActor;

        /// <summary>
        /// Physical scene representation of this Avatar.
        /// </summary>
        public PhysicsActor PhysicsActor
        {
            set { m_physicsActor = value; }
            get { return m_physicsActor; }
        }

        private byte m_movementflag;

        public byte MovementFlag
        {
            set { m_movementflag = value; }
            get { return m_movementflag; }
        }

        private bool m_updateflag;

        public bool Updated
        {
            set { m_updateflag = value; }
            get { return m_updateflag; }
        }

        private bool m_invulnerable = true;

        public bool Invulnerable
        {
            set { m_invulnerable = value; }
            get { return m_invulnerable; }
        }

        private int m_userLevel;

        public int UserLevel
        {
            get { return m_userLevel; }
            private set { m_userLevel = value; }
        }

        private int m_godLevel;

        public int GodLevel
        {
            get { return m_godLevel; }
            private set { m_godLevel = value; }
        }

        private ulong m_rootRegionHandle;

        public ulong RegionHandle
        {
            get { return m_rootRegionHandle; }
            private set { m_rootRegionHandle = value; }
        }

        #region Client Camera

        /// <summary>
        /// Position of agent's camera in world (region cordinates)
        /// </summary>
        protected Vector3 m_lastCameraPosition;

        protected Vector3 m_CameraPosition;

        public Vector3 CameraPosition
        {
            get { return m_CameraPosition; }
            private set { m_CameraPosition = value; }
        }

        public Quaternion CameraRotation
        {
            get { return Util.Axes2Rot(m_CameraAtAxis, m_CameraLeftAxis, m_CameraUpAxis); }
        }

        // Use these three vectors to figure out what the agent is looking at
        // Convert it to a Matrix and/or Quaternion
        //
        protected Vector3 m_CameraAtAxis;
        protected Vector3 m_CameraLeftAxis;
        protected Vector3 m_CameraUpAxis;

        public Vector3 CameraAtAxis
        {
            get { return m_CameraAtAxis; }
            private set { m_CameraAtAxis = value; }
        }


        public Vector3 CameraLeftAxis
        {
            get { return m_CameraLeftAxis; }
            private set { m_CameraLeftAxis = value; }
        }

        public Vector3 CameraUpAxis
        {
            get { return m_CameraUpAxis; }
            private set { m_CameraUpAxis = value; }
        }

        public Vector3 Lookat
        {
            get
            {
                Vector3 a = new Vector3(CameraAtAxis.X, CameraAtAxis.Y, 0);

                if (a == Vector3.Zero)
                    return a;

                return Util.GetNormalizedVector(a);
            }
        }
        #endregion        

        public readonly string Firstname;
        public readonly string Lastname;

        private string m_grouptitle;

        public string Grouptitle
        {
            get { return m_grouptitle; }
            set { m_grouptitle = value; }
        }

        // Agent's Draw distance.
        protected float m_DrawDistance;

        public float DrawDistance
        {
            get { return m_DrawDistance; }
            private set { m_DrawDistance = value; }
        }

        protected bool m_allowMovement = true;

        public bool AllowMovement
        {
            get { return m_allowMovement; }
            set { m_allowMovement = value; }
        }

        private bool m_setAlwaysRun;
        
        public bool SetAlwaysRun
        {
            get
            {
                if (PhysicsActor != null)
                {
                    return PhysicsActor.SetAlwaysRun;
                }
                else
                {
                    return m_setAlwaysRun;
                }
            }
            set
            {
                m_setAlwaysRun = value;
                if (PhysicsActor != null)
                {
                    PhysicsActor.SetAlwaysRun = value;
                }
            }
        }

        private byte m_state;

        public byte State
        {
            get { return m_state; }
            set { m_state = value; }
        }

        private AgentManager.ControlFlags m_AgentControlFlags;

        public uint AgentControlFlags
        {
            get { return (uint)m_AgentControlFlags; }
            set { m_AgentControlFlags = (AgentManager.ControlFlags)value; }
        }

        /// <summary>
        /// This works out to be the ClientView object associated with this avatar, or it's client connection manager
        /// </summary>
        private IClientAPI m_controllingClient;

        public IClientAPI ControllingClient
        {
            get { return m_controllingClient; }
            private set { m_controllingClient = value; }
        }

        public IClientCore ClientView
        {
            get { return (IClientCore) m_controllingClient; }
        }

        protected Vector3 m_parentPosition;

        protected SceneObjectPart m_sitPart = null;

        /// <summary>
        /// Position of this avatar relative to the region the avatar is in
        /// </summary>
        public override Vector3 AbsolutePosition
        {
            get
            {
                if (PhysicsActor != null && m_parentID == 0)
                {
                    m_pos = PhysicsActor.Position;

                    //                    m_log.DebugFormat(
                    //                        "[SCENE PRESENCE]: Set position {0} for {1} in {2} via getting AbsolutePosition!",
                    //                        m_pos, Name, Scene.RegionInfo.RegionName);
                }
                else
                {
                    // Obtain the correct position of a seated avatar.
                    // In addition to providing the correct position while
                    // the avatar is seated, this value will also
                    // be used as the location to unsit to.
   
                    if (m_sitPart != null)
                    {
                        return m_sitPart.AbsolutePosition + (m_pos * m_sitPart.RotationOffset);
                    }
                    else
                    {
                        return m_pos;
                    }
                }
                return m_pos;
            }
            set
            {
                if (PhysicsActor != null)
                {
                    try
                    {
                        PhysicsActor.Position = value;
                    }
                    catch (Exception e)
                    {
                        m_log.Error("[SCENE PRESENCE]: ABSOLUTE POSITION " + e.Message);
                    }
                }

                m_pos = value;

//                m_log.DebugFormat(
//                    "[ENTITY BASE]: In {0} set AbsolutePosition of {1} to {2}",
//                    Scene.RegionInfo.RegionName, Name, m_pos);
            }
        }

        /// <summary>
        /// If sitting, returns the offset position from the prim the avatar is sitting on.
        /// Otherwise, returns absolute position in the scene.
        /// </summary>
        public Vector3 OffsetPosition
        {
            get
            {
                if (m_sitPart != null)
                {
                    return m_sitPart.OffsetPosition + (m_pos * m_sitPart.RotationOffset);
                }
                else
                    return m_pos;
            }
        }

        /// <summary>
        /// Current velocity of the avatar.
        /// </summary>
        public override Vector3 Velocity
        {
            get
            {
                if (PhysicsActor != null)
                {
                    m_velocity = PhysicsActor.Velocity;

//                    m_log.DebugFormat(
//                        "[SCENE PRESENCE]: Set velocity {0} for {1} in {2} via getting Velocity!",
//                        m_velocity, Name, Scene.RegionInfo.RegionName);
                }

                return m_velocity;
            }
            set
            {
                if (PhysicsActor != null)
                {
                    try
                    {
                        PhysicsActor.Velocity = value;
                    }
                    catch (Exception e)
                    {
                        m_log.Error("[SCENE PRESENCE]: VELOCITY " + e.Message);
                    }
                }

                m_velocity = value;

//                m_log.DebugFormat(
//                    "[SCENE PRESENCE]: In {0} set velocity of {1} to {2}",
//                    Scene.RegionInfo.RegionName, Name, m_velocity);
            }
        }

        private Quaternion m_bodyRot = Quaternion.Identity;

        public Quaternion Rotation
        {
            get
            {
                if (m_sitPart != null)
                {
                    return m_bodyRot * m_sitPart.RotationOffset;
                }
                else
                    return m_bodyRot;
            }
            set
            {
                m_bodyRot = value;
//                m_log.DebugFormat("[SCENE PRESENCE]: Body rot for {0} set to {1}", Name, m_bodyRot);
            }
        }

        /// <summary>
        /// If this is true, agent doesn't have a representation in this scene.
        ///    this is an agent 'looking into' this scene from a nearby scene(region)
        ///
        /// if False, this agent has a representation in this scene
        /// </summary>
        private bool m_isChildAgent = true;

        public bool IsChildAgent
        {
            get { return m_isChildAgent; }
            set { m_isChildAgent = value; }
        }

        private uint m_parentID = 0;
        public uint ParentID // id of object we are sitting on
        {
            get { return m_parentID; }
            set { m_parentID = value; }
        }

        private uint m_SitPartID = 0;
        public uint SitPartID // id of part we are sitting on
        {
            get { return m_SitPartID; }
            set { m_SitPartID = value; }
        }

        public float Health
        {
            get { return m_health; }
            set { m_health = value; }
        }

        public void AdjustKnownSeeds()
        {
            Dictionary<ulong, string> seeds;

            if (Scene.CapsModule != null)
                seeds = Scene.CapsModule.GetChildrenSeeds(UUID);
            else
                seeds = new Dictionary<ulong, string>();

            List<ulong> old = new List<ulong>();
            foreach (ulong handle in seeds.Keys)
            {
                uint x, y;
                Utils.LongToUInts(handle, out x, out y);
                x = x / Constants.RegionSize;
                y = y / Constants.RegionSize;
                if (Util.IsOutsideView(DrawDistance, x, Scene.RegionInfo.RegionLocX, y, Scene.RegionInfo.RegionLocY))
                {
                    old.Add(handle);
                }
            }
            DropOldNeighbours(old);
            
            if (Scene.CapsModule != null)
                Scene.CapsModule.SetChildrenSeed(UUID, seeds);
            
            KnownRegions = seeds;
            //m_log.Debug(" ++++++++++AFTER+++++++++++++ ");
            //DumpKnownRegions();
        }

        public void DumpKnownRegions()
        {
            m_log.Info("================ KnownRegions "+Scene.RegionInfo.RegionName+" ================");
            foreach (KeyValuePair<ulong, string> kvp in KnownRegions)
            {
                uint x, y;
                Utils.LongToUInts(kvp.Key, out x, out y);
                x = x / Constants.RegionSize;
                y = y / Constants.RegionSize;
                m_log.Info(" >> "+x+", "+y+": "+kvp.Value);
            }
        }

        private bool m_mouseLook;
        private bool m_leftButtonDown;

        private bool m_inTransit;

        public bool IsInTransit
        {
            get { return m_inTransit; }
            set { 
                if(value)
                {
                    if ((PhysicsActor != null) && PhysicsActor.Flying)
                        m_AgentControlFlags |= AgentManager.ControlFlags.AGENT_CONTROL_FLY;
                    else if ((m_AgentControlFlags & AgentManager.ControlFlags.AGENT_CONTROL_FLY) != 0)
                        m_AgentControlFlags &= ~AgentManager.ControlFlags.AGENT_CONTROL_FLY;
        }
                m_inTransit = value;
            }
        }

        private float m_speedModifier = 1.0f;

        public float SpeedModifier
        {
            get { return m_speedModifier; }
            set { m_speedModifier = value; }
        }

        private bool m_forceFly;

        public bool ForceFly
        {
            get { return m_forceFly; }
            set { m_forceFly = value; }
        }

        private bool m_flyDisabled;

        public bool FlyDisabled
        {
            get { return m_flyDisabled; }
            set { m_flyDisabled = value; }
        }

        public string Viewer
        {
            get { return m_scene.AuthenticateHandler.GetAgentCircuitData(ControllingClient.CircuitCode).Viewer; }
        }

        #endregion

        #region Constructor(s)

        public ScenePresence(
            IClientAPI client, Scene world, AvatarAppearance appearance, PresenceType type)
        {
            AttachmentsSyncLock = new Object();

            m_sendCourseLocationsMethod = SendCoarseLocationsDefault;
            Animator = new ScenePresenceAnimator(this);
            PresenceType = type;
            DrawDistance = world.DefaultDrawDistance;
            RegionHandle = world.RegionInfo.RegionHandle;
            ControllingClient = client;
            Firstname = ControllingClient.FirstName;
            Lastname = ControllingClient.LastName;
            m_name = String.Format("{0} {1}", Firstname, Lastname);
            m_scene = world;
            m_uuid = client.AgentId;
            LocalId = m_scene.AllocateLocalId();

            UserAccount account = m_scene.UserAccountService.GetUserAccount(m_scene.RegionInfo.ScopeID, m_uuid);
            if (account != null)
                m_userFlags = account.UserFlags;
            else
                m_userFlags = 0;

            if (account != null)
                UserLevel = account.UserLevel;

            IGroupsModule gm = m_scene.RequestModuleInterface<IGroupsModule>();
            if (gm != null)
                Grouptitle = gm.GetGroupTitle(m_uuid);

            m_scriptEngines = m_scene.RequestModuleInterfaces<IScriptModule>();
            
            AbsolutePosition = posLastSignificantMove = CameraPosition =
                m_lastCameraPosition = ControllingClient.StartPos;

            m_reprioritization_timer = new Timer(world.ReprioritizationInterval);
            m_reprioritization_timer.Elapsed += new ElapsedEventHandler(Reprioritize);
            m_reprioritization_timer.AutoReset = false;

            AdjustKnownSeeds();

            RegisterToEvents();
            SetDirectionVectors();

            Appearance = appearance;
        }

        public void RegisterToEvents()
        {
            ControllingClient.OnCompleteMovementToRegion += CompleteMovement;
            //ControllingClient.OnCompleteMovementToRegion += SendInitialData;
            ControllingClient.OnAgentUpdate += HandleAgentUpdate;
            ControllingClient.OnAgentRequestSit += HandleAgentRequestSit;
            ControllingClient.OnAgentSit += HandleAgentSit;
            ControllingClient.OnSetAlwaysRun += HandleSetAlwaysRun;
            ControllingClient.OnStartAnim += HandleStartAnim;
            ControllingClient.OnStopAnim += HandleStopAnim;
            ControllingClient.OnForceReleaseControls += HandleForceReleaseControls;
            ControllingClient.OnAutoPilotGo += MoveToTarget;

            // ControllingClient.OnChildAgentStatus += new StatusChange(this.ChildStatusChange);
            // ControllingClient.OnStopMovement += new GenericCall2(this.StopMovement);
        }

        private void SetDirectionVectors()
        {
            Dir_Vectors[0] = Vector3.UnitX; //FORWARD
            Dir_Vectors[1] = -Vector3.UnitX; //BACK
            Dir_Vectors[2] = Vector3.UnitY; //LEFT
            Dir_Vectors[3] = -Vector3.UnitY; //RIGHT
            Dir_Vectors[4] = Vector3.UnitZ; //UP
            Dir_Vectors[5] = -Vector3.UnitZ; //DOWN
            Dir_Vectors[6] = new Vector3(0.5f, 0f, 0f); //FORWARD_NUDGE
            Dir_Vectors[7] = new Vector3(-0.5f, 0f, 0f);  //BACK_NUDGE
            Dir_Vectors[8] = new Vector3(0f, 0.5f, 0f);  //LEFT_NUDGE
            Dir_Vectors[9] = new Vector3(0f, -0.5f, 0f);  //RIGHT_NUDGE
            Dir_Vectors[10] = new Vector3(0f, 0f, -0.5f); //DOWN_Nudge
        }

        private Vector3[] GetWalkDirectionVectors()
        {
            Vector3[] vector = new Vector3[11];
            vector[0] = new Vector3(m_CameraUpAxis.Z, 0f, -m_CameraAtAxis.Z); //FORWARD
            vector[1] = new Vector3(-m_CameraUpAxis.Z, 0f, m_CameraAtAxis.Z); //BACK
            vector[2] = Vector3.UnitY; //LEFT
            vector[3] = -Vector3.UnitY; //RIGHT
            vector[4] = new Vector3(m_CameraAtAxis.Z, 0f, m_CameraUpAxis.Z); //UP
            vector[5] = new Vector3(-m_CameraAtAxis.Z, 0f, -m_CameraUpAxis.Z); //DOWN
            vector[6] = new Vector3(m_CameraUpAxis.Z, 0f, -m_CameraAtAxis.Z); //FORWARD_NUDGE
            vector[7] = new Vector3(-m_CameraUpAxis.Z, 0f, m_CameraAtAxis.Z); //BACK_NUDGE
            vector[8] = Vector3.UnitY; //LEFT_NUDGE
            vector[9] = -Vector3.UnitY; //RIGHT_NUDGE
            vector[10] = new Vector3(-m_CameraAtAxis.Z, 0f, -m_CameraUpAxis.Z); //DOWN_NUDGE
            return vector;
        }

        #endregion

        public uint GenerateClientFlags(UUID ObjectID)
        {
            return m_scene.Permissions.GenerateClientFlags(m_uuid, ObjectID);
        }

        #region Status Methods

        /// <summary>
        /// Turns a child agent into a root agent.
        /// </summary>
        /// Child agents are logged into neighbouring sims largely to observe changes.  Root agents exist when the
        /// avatar is actual in the sim.  They can perform all actions.
        /// This change is made whenever an avatar enters a region, whether by crossing over from a neighbouring sim,
        /// teleporting in or on initial login.
        ///
        /// This method is on the critical path for transferring an avatar from one region to another.  Delay here
        /// delays that crossing.
        /// </summary>
        public void MakeRootAgent(Vector3 pos, bool isFlying)
        {
            m_log.DebugFormat(
                "[SCENE]: Upgrading child to root agent for {0} in {1}",
                Name, m_scene.RegionInfo.RegionName);

            //m_log.DebugFormat("[SCENE]: known regions in {0}: {1}", Scene.RegionInfo.RegionName, KnownChildRegionHandles.Count);

            bool wasChild = IsChildAgent;
            IsChildAgent = false;

            IGroupsModule gm = m_scene.RequestModuleInterface<IGroupsModule>();
            if (gm != null)
                Grouptitle = gm.GetGroupTitle(m_uuid);

            RegionHandle = m_scene.RegionInfo.RegionHandle;

            m_scene.EventManager.TriggerSetRootAgentScene(m_uuid, m_scene);

            // Moved this from SendInitialData to ensure that Appearance is initialized
            // before the inventory is processed in MakeRootAgent. This fixes a race condition
            // related to the handling of attachments
            //m_scene.GetAvatarAppearance(ControllingClient, out Appearance);
            if (m_scene.TestBorderCross(pos, Cardinals.E))
            {
                Border crossedBorder = m_scene.GetCrossedBorder(pos, Cardinals.E);
                pos.X = crossedBorder.BorderLine.Z - 1;
            }

            if (m_scene.TestBorderCross(pos, Cardinals.N))
            {
                Border crossedBorder = m_scene.GetCrossedBorder(pos, Cardinals.N);
                pos.Y = crossedBorder.BorderLine.Z - 1;
            }

            if (pos.X < 0f || pos.Y < 0f || pos.Z < 0f)
            {
                m_log.WarnFormat(
                    "[SCENE PRESENCE]: MakeRootAgent() was given an illegal position of {0} for avatar {1}, {2}. Clamping",
                    pos, Name, UUID);

                if (pos.X < 0f) pos.X = 0f;
                if (pos.Y < 0f) pos.Y = 0f;
                if (pos.Z < 0f) pos.Z = 0f;
            }

            float localAVHeight = 1.56f;
            if (Appearance.AvatarHeight > 0)
                localAVHeight = Appearance.AvatarHeight;

            float posZLimit = 0;

            if (pos.X < Constants.RegionSize && pos.Y < Constants.RegionSize)
                posZLimit = (float)m_scene.Heightmap[(int)pos.X, (int)pos.Y];
            
            float newPosZ = posZLimit + localAVHeight / 2;
            if (posZLimit >= (pos.Z - (localAVHeight / 2)) && !(Single.IsInfinity(newPosZ) || Single.IsNaN(newPosZ)))
            {
                pos.Z = newPosZ;
            }
            AbsolutePosition = pos;

            AddToPhysicalScene(isFlying);

            if (ForceFly)
            {
                PhysicsActor.Flying = true;
            }
            else if (FlyDisabled)
            {
                PhysicsActor.Flying = false;
            }

            // Don't send an animation pack here, since on a region crossing this will sometimes cause a flying 
            // avatar to return to the standing position in mid-air.  On login it looks like this is being sent
            // elsewhere anyway
            // Animator.SendAnimPack();

            m_scene.SwapRootAgentCount(false);

            // The initial login scene presence is already root when it gets here
            // and it has already rezzed the attachments and started their scripts.
            // We do the following only for non-login agents, because their scripts
            // haven't started yet.
            lock (m_attachments)
            {
                if (wasChild && HasAttachments())
                {
                    m_log.DebugFormat("[SCENE PRESENCE]: Restarting scripts in attachments...");
                    // Resume scripts
                    foreach (SceneObjectGroup sog in m_attachments)
                    {
                        sog.RootPart.ParentGroup.CreateScriptInstances(0, false, m_scene.DefaultScriptEngine, GetStateSource());
                        sog.ResumeScripts();
                    }
                }
            }

            // send the animations of the other presences to me
            m_scene.ForEachRootScenePresence(delegate(ScenePresence presence)
            {
                if (presence != this)
                    presence.Animator.SendAnimPackToClient(ControllingClient);
            });

            // If we don't reset the movement flag here, an avatar that crosses to a neighbouring sim and returns will
            // stall on the border crossing since the existing child agent will still have the last movement
            // recorded, which stops the input from being processed.
            MovementFlag = 0;

            m_scene.EventManager.TriggerOnMakeRootAgent(this);
        }

        public int GetStateSource()
        {
            AgentCircuitData aCircuit = m_scene.AuthenticateHandler.GetAgentCircuitData(UUID);

            if (aCircuit != null && (aCircuit.teleportFlags != (uint)TeleportFlags.Default))
            {
                // This will get your attention
                //m_log.Error("[XXX] Triggering CHANGED_TELEPORT");

                return 5; // StateSource.Teleporting
            }
            return 2; // StateSource.PrimCrossing
        }

        /// <summary>
        /// This turns a root agent into a child agent
        /// </summary>
        /// <remarks>
        /// when an agent departs this region for a neighbor, this gets called.
        ///
        /// It doesn't get called for a teleport.  Reason being, an agent that
        /// teleports out may not end up anywhere near this region
        /// </remarks>
        public void MakeChildAgent()
        {
            m_log.DebugFormat("[SCENE PRESENCE]: Making {0} a child agent in {1}", Name, Scene.RegionInfo.RegionName);

            // Reset these so that teleporting in and walking out isn't seen
            // as teleporting back
            TeleportFlags = TeleportFlags.Default;

            // It looks like Animator is set to null somewhere, and MakeChild
            // is called after that. Probably in aborted teleports.
            if (Animator == null)
                Animator = new ScenePresenceAnimator(this);
            else
                Animator.ResetAnimations();

//            m_log.DebugFormat(
//                 "[SCENE PRESENCE]: Downgrading root agent {0}, {1} to a child agent in {2}",
//                 Name, UUID, m_scene.RegionInfo.RegionName);

            // Don't zero out the velocity since this can cause problems when an avatar is making a region crossing,
            // depending on the exact timing.  This shouldn't matter anyway since child agent positions are not updated.
            //Velocity = new Vector3(0, 0, 0);

            IsChildAgent = true;
            m_scene.SwapRootAgentCount(true);
            RemoveFromPhysicalScene();

            // FIXME: Set RegionHandle to the region handle of the scene this agent is moving into
            
            m_scene.EventManager.TriggerOnMakeChildAgent(this);
        }

        /// <summary>
        /// Removes physics plugin scene representation of this agent if it exists.
        /// </summary>
        public void RemoveFromPhysicalScene()
        {
            if (PhysicsActor != null)
            {
                try
                {
                    PhysicsActor.OnRequestTerseUpdate -= SendTerseUpdateToAllClients;
                    PhysicsActor.OnOutOfBounds -= OutOfBoundsCall;
                    m_scene.PhysicsScene.RemoveAvatar(PhysicsActor);
                    PhysicsActor.UnSubscribeEvents();
                    PhysicsActor.OnCollisionUpdate -= PhysicsCollisionUpdate;
                    PhysicsActor = null;
                }
                catch
                { }
            }
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="pos"></param>
        public void Teleport(Vector3 pos)
        {
            bool isFlying = false;
            if (PhysicsActor != null)
                isFlying = PhysicsActor.Flying;
            
            RemoveFromPhysicalScene();
            Velocity = Vector3.Zero;
            AbsolutePosition = pos;
            AddToPhysicalScene(isFlying);

            SendTerseUpdateToAllClients();
        }

        public void TeleportWithMomentum(Vector3 pos)
        {
            bool isFlying = false;
            Vector3 velocity = Vector3.Zero;

            if (PhysicsActor != null)
            {
                isFlying = PhysicsActor.Flying;
                velocity = PhysicsActor.Velocity;
            }

            RemoveFromPhysicalScene();
            AbsolutePosition = pos;
            AddToPhysicalScene(isFlying);
            if (PhysicsActor != null)
                PhysicsActor.Velocity = velocity;

            SendTerseUpdateToAllClients();
        }

        public void StopFlying()
        {
            ControllingClient.StopFlying(this);
        }

        // neighbouring regions we have enabled a child agent in
        // holds the seed cap for the child agent in that region
        private Dictionary<ulong, string> m_knownChildRegions = new Dictionary<ulong, string>();

        public void AddNeighbourRegion(ulong regionHandle, string cap)
        {
            lock (m_knownChildRegions)
            {
                if (!m_knownChildRegions.ContainsKey(regionHandle))
                {
                    uint x, y;
                    Utils.LongToUInts(regionHandle, out x, out y);
                    m_knownChildRegions.Add(regionHandle, cap);
                }
            }
        }

        public void RemoveNeighbourRegion(ulong regionHandle)
        {
            lock (m_knownChildRegions)
            {
                // Checking ContainsKey is redundant as Remove works either way and returns a bool
                // This is here to allow the Debug output to be conditional on removal
                //if (m_knownChildRegions.ContainsKey(regionHandle))
                //    m_log.DebugFormat(" !!! removing known region {0} in {1}. Count = {2}", regionHandle, Scene.RegionInfo.RegionName, m_knownChildRegions.Count);
                m_knownChildRegions.Remove(regionHandle);
                }
            }

        public void DropOldNeighbours(List<ulong> oldRegions)
        {
            foreach (ulong handle in oldRegions)
            {
                RemoveNeighbourRegion(handle);
                Scene.CapsModule.DropChildSeed(UUID, handle);
            }
        }

        public Dictionary<ulong, string> KnownRegions
        {
            get
            {
                lock (m_knownChildRegions)
                    return new Dictionary<ulong, string>(m_knownChildRegions);
        }
            set
            {
                // Replacing the reference is atomic but we still need to lock on
                // the original dictionary object which may be in use elsewhere
                lock (m_knownChildRegions)
                    m_knownChildRegions = value;
            }
        }

        public List<ulong> KnownRegionHandles
        {
            get
            {
                return new List<ulong>(KnownRegions.Keys);
            }
        }

        public int KnownRegionCount
        {
            get
            {
                lock (m_knownChildRegions)
                    return m_knownChildRegions.Count;
            }
        }

        #endregion

        #region Event Handlers

        /// <summary>
        /// Sets avatar height in the physics plugin
        /// </summary>
        public void SetHeight(float height)
        {
            if (PhysicsActor != null && !IsChildAgent)
            {
                Vector3 SetSize = new Vector3(0.45f, 0.6f, height);
                PhysicsActor.Size = SetSize;
            }
        }

        /// <summary>
        /// Complete Avatar's movement into the region.
        /// </summary>
        /// <param name="client"></param>
        /// <param name="openChildAgents">
        /// If true, send notification to neighbour regions to expect
        /// a child agent from the client.  These neighbours can be some distance away, depending right now on the
        /// configuration of DefaultDrawDistance in the [Startup] section of config
        /// </param>
        public void CompleteMovement(IClientAPI client, bool openChildAgents)
        {
//            DateTime startTime = DateTime.Now;
            
//            m_log.DebugFormat(
//                "[SCENE PRESENCE]: Completing movement of {0} into region {1}", 
//                client.Name, Scene.RegionInfo.RegionName);

            Vector3 look = Velocity;
            if ((look.X == 0) && (look.Y == 0) && (look.Z == 0))
            {
                look = new Vector3(0.99f, 0.042f, 0);
            }

            // Prevent teleporting to an underground location
            // (may crash client otherwise)
            //
            Vector3 pos = AbsolutePosition;
            float ground = m_scene.GetGroundHeight(pos.X, pos.Y);
            if (pos.Z < ground + 1.5f)
            {
                pos.Z = ground + 1.5f;
                AbsolutePosition = pos;
            }

            bool m_flying = ((m_AgentControlFlags & AgentManager.ControlFlags.AGENT_CONTROL_FLY) != 0);
            MakeRootAgent(AbsolutePosition, m_flying);

            if ((m_callbackURI != null) && !m_callbackURI.Equals(""))
            {
                m_log.DebugFormat("[SCENE PRESENCE]: Releasing agent in URI {0}", m_callbackURI);
                Scene.SimulationService.ReleaseAgent(m_originRegionID, UUID, m_callbackURI);
                m_callbackURI = null;
            }

            //m_log.DebugFormat("[SCENE PRESENCE] Completed movement");

            ControllingClient.MoveAgentIntoRegion(m_scene.RegionInfo, AbsolutePosition, look);
            ValidateAndSendAppearanceAndAgentData();

            // Create child agents in neighbouring regions
            if (openChildAgents && !IsChildAgent)
            {
                IEntityTransferModule m_agentTransfer = m_scene.RequestModuleInterface<IEntityTransferModule>();
                if (m_agentTransfer != null)
                    m_agentTransfer.EnableChildAgents(this);

                IFriendsModule friendsModule = m_scene.RequestModuleInterface<IFriendsModule>();
                if (friendsModule != null)
                    friendsModule.SendFriendsOnlineIfNeeded(ControllingClient);
            }

            if (m_scene.RegionInfo.CombinedRegionHandle != 0)
                CheckForBorderCrossing();

//            m_log.DebugFormat(
//                "[SCENE PRESENCE]: Completing movement of {0} into region {1} took {2}ms", 
//                client.Name, Scene.RegionInfo.RegionName, (DateTime.Now - startTime).Milliseconds);
        }

        /// <summary>
        /// Callback for the Camera view block check.  Gets called with the results of the camera view block test
        /// hitYN is true when there's something in the way.
        /// </summary>
        /// <param name="hitYN"></param>
        /// <param name="collisionPoint"></param>
        /// <param name="localid"></param>
        /// <param name="distance"></param>
        public void RayCastCameraCallback(bool hitYN, Vector3 collisionPoint, uint localid, float distance, Vector3 pNormal)
        {
            const float POSITION_TOLERANCE = 0.02f;
            const float VELOCITY_TOLERANCE = 0.02f;
            const float ROTATION_TOLERANCE = 0.02f;

            if (m_followCamAuto)
            {
                if (hitYN)
                {
                    CameraConstraintActive = true;
                    //m_log.DebugFormat("[RAYCASTRESULT]: {0}, {1}, {2}, {3}", hitYN, collisionPoint, localid, distance);
                    
                    Vector3 normal = Vector3.Normalize(new Vector3(0f, 0f, collisionPoint.Z) - collisionPoint);
                    ControllingClient.SendCameraConstraint(new Vector4(normal.X, normal.Y, normal.Z, -1 * Vector3.Distance(new Vector3(0,0,collisionPoint.Z),collisionPoint)));
                }
                else
                {
                    if (!m_pos.ApproxEquals(m_lastPosition, POSITION_TOLERANCE) ||
                        !Velocity.ApproxEquals(m_lastVelocity, VELOCITY_TOLERANCE) ||
                        !Rotation.ApproxEquals(m_lastRotation, ROTATION_TOLERANCE))
                    {
                        if (CameraConstraintActive)
                        {
                            ControllingClient.SendCameraConstraint(new Vector4(0f, 0.5f, 0.9f, -3000f));
                            CameraConstraintActive = false;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// This is the event handler for client movement. If a client is moving, this event is triggering.
        /// </summary>
        public void HandleAgentUpdate(IClientAPI remoteClient, AgentUpdateArgs agentData)
        {
//            m_log.DebugFormat(
//                "[SCENE PRESENCE]: In {0} received agent update from {1}",
//                Scene.RegionInfo.RegionName, remoteClient.Name);

            if (IsChildAgent)
            {
            //    // m_log.Debug("DEBUG: HandleAgentUpdate: child agent");
                return;
            }

            ++m_movementUpdateCount;
            if (m_movementUpdateCount < 1)
                m_movementUpdateCount = 1;

            #region Sanity Checking

            // This is irritating.  Really.
            if (!AbsolutePosition.IsFinite())
            {
                RemoveFromPhysicalScene();
                m_log.Error("[AVATAR]: NonFinite Avatar position detected... Reset Position. Mantis this please. Error #9999902");

                m_pos = m_LastFinitePos;
                if (!m_pos.IsFinite())
                {
                    m_pos.X = 127f;
                    m_pos.Y = 127f;
                    m_pos.Z = 127f;
                    m_log.Error("[AVATAR]: NonFinite Avatar position detected... Reset Position. Mantis this please. Error #9999903");
                }

                AddToPhysicalScene(false);
            }
            else
            {
                m_LastFinitePos = m_pos;
            }

            #endregion Sanity Checking

            #region Inputs

            AgentManager.ControlFlags flags = (AgentManager.ControlFlags)agentData.ControlFlags;

            // Camera location in world.  We'll need to raytrace
            // from this location from time to time.
            CameraPosition = agentData.CameraCenter;
            if (Vector3.Distance(m_lastCameraPosition, CameraPosition) >= Scene.RootReprioritizationDistance)
            {
                ReprioritizeUpdates();
                m_lastCameraPosition = CameraPosition;
            }

            // Use these three vectors to figure out what the agent is looking at
            // Convert it to a Matrix and/or Quaternion
            CameraAtAxis = agentData.CameraAtAxis;
            CameraLeftAxis = agentData.CameraLeftAxis;
            m_CameraUpAxis = agentData.CameraUpAxis;

            // The Agent's Draw distance setting
            // When we get to the point of re-computing neighbors everytime this
            // changes, then start using the agent's drawdistance rather than the 
            // region's draw distance.
            // DrawDistance = agentData.Far;
            DrawDistance = Scene.DefaultDrawDistance;

            // Check if Client has camera in 'follow cam' or 'build' mode.
            Vector3 camdif = (Vector3.One * Rotation - Vector3.One * CameraRotation);

            m_followCamAuto = ((m_CameraUpAxis.Z > 0.959f && m_CameraUpAxis.Z < 0.98f)
               && (Math.Abs(camdif.X) < 0.4f && Math.Abs(camdif.Y) < 0.4f)) ? true : false;

            m_mouseLook = (flags & AgentManager.ControlFlags.AGENT_CONTROL_MOUSELOOK) != 0;
            m_leftButtonDown = (flags & AgentManager.ControlFlags.AGENT_CONTROL_LBUTTON_DOWN) != 0;

            #endregion Inputs

//            // Make anims work for client side autopilot
//            if ((flags & AgentManager.ControlFlags.AGENT_CONTROL_AT_POS) != 0)
//                m_updateCount = UPDATE_COUNT;
//
//            // Make turning in place work
//            if ((flags & AgentManager.ControlFlags.AGENT_CONTROL_YAW_POS) != 0 ||
//                (flags & AgentManager.ControlFlags.AGENT_CONTROL_YAW_NEG) != 0)
//                m_updateCount = UPDATE_COUNT;

            if ((flags & AgentManager.ControlFlags.AGENT_CONTROL_STAND_UP) != 0)
            {
                StandUp();
            }

            //m_log.DebugFormat("[FollowCam]: {0}", m_followCamAuto);
            // Raycast from the avatar's head to the camera to see if there's anything blocking the view
            if ((m_movementUpdateCount % NumMovementsBetweenRayCast) == 0 && m_scene.PhysicsScene.SupportsRayCast())
            {
                if (m_followCamAuto)
                {
                    Vector3 posAdjusted = m_pos + HEAD_ADJUSTMENT;
                    m_scene.PhysicsScene.RaycastWorld(m_pos, Vector3.Normalize(CameraPosition - posAdjusted), Vector3.Distance(CameraPosition, posAdjusted) + 0.3f, RayCastCameraCallback);
                }
            }

            lock (scriptedcontrols)
            {
                if (scriptedcontrols.Count > 0)
                {
                    SendControlToScripts((uint)flags);
                    flags = RemoveIgnoredControls(flags, IgnoredControls);
                }
            }

            if ((flags & AgentManager.ControlFlags.AGENT_CONTROL_SIT_ON_GROUND) != 0)
                HandleAgentSitOnGround();

            // In the future, these values might need to go global.
            // Here's where you get them.
            m_AgentControlFlags = flags;
            m_headrotation = agentData.HeadRotation;
            State = agentData.State;

            PhysicsActor actor = PhysicsActor;
            if (actor == null)
            {
                return;
            }

            if (AllowMovement && !SitGround)
            {
                Quaternion bodyRotation = agentData.BodyRotation;
                bool update_rotation = false;

                if (bodyRotation != m_bodyRot)
                {
                    Rotation = bodyRotation;
                    update_rotation = true;
                }

                bool update_movementflag = false;

                if (agentData.UseClientAgentPosition)
                {
                    MovingToTarget = (agentData.ClientAgentPosition - AbsolutePosition).Length() > 0.2f;
                    MoveToPositionTarget = agentData.ClientAgentPosition;
                }

                int i = 0;
                bool DCFlagKeyPressed = false;
                Vector3 agent_control_v3 = Vector3.Zero;

                bool oldflying = PhysicsActor.Flying;

                if (ForceFly)
                    actor.Flying = true;
                else if (FlyDisabled)
                    actor.Flying = false;
                else
                    actor.Flying = ((flags & AgentManager.ControlFlags.AGENT_CONTROL_FLY) != 0);

                if (actor.Flying != oldflying)
                    update_movementflag = true;

                if (ParentID == 0)
                {
                    bool bAllowUpdateMoveToPosition = false;

                    Vector3[] dirVectors;

                    // use camera up angle when in mouselook and not flying or when holding the left mouse button down and not flying
                    // this prevents 'jumping' in inappropriate situations.
                    if ((m_mouseLook && !PhysicsActor.Flying) || (m_leftButtonDown && !PhysicsActor.Flying))
                        dirVectors = GetWalkDirectionVectors();
                    else
                        dirVectors = Dir_Vectors;

                    // The fact that MovementFlag is a byte needs to be fixed
                    // it really should be a uint
                    // A DIR_CONTROL_FLAG occurs when the user is trying to move in a particular direction.
                    uint nudgehack = 250;
                    foreach (Dir_ControlFlags DCF in DIR_CONTROL_FLAGS)
                    {
                        if (((uint)flags & (uint)DCF) != 0)
                        {
                            DCFlagKeyPressed = true;

                            try
                            {
                                agent_control_v3 += dirVectors[i];
                                //m_log.DebugFormat("[Motion]: {0}, {1}",i, dirVectors[i]);
                            }
                            catch (IndexOutOfRangeException)
                            {
                                // Why did I get this?
                            }

                            if ((MovementFlag & (byte)(uint)DCF) == 0)
                            {
                                if (DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_FORWARD_NUDGE || DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_BACKWARD_NUDGE ||
                                    DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_LEFT_NUDGE || DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_RIGHT_NUDGE)
                                {
                                    MovementFlag |= (byte)nudgehack;
                                }

//                                m_log.DebugFormat("[SCENE PRESENCE]: Updating MovementFlag for {0} with {1}", Name, DCF);
                                MovementFlag += (byte)(uint)DCF;
                                update_movementflag = true;
                            }
                        }
                        else
                        {
                            if ((MovementFlag & (byte)(uint)DCF) != 0 ||
                                ((DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_FORWARD_NUDGE || DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_BACKWARD_NUDGE ||
                                DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_LEFT_NUDGE || DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_RIGHT_NUDGE)
                                && ((MovementFlag & (byte)nudgehack) == nudgehack))
                                ) // This or is for Nudge forward
                            {
//                                m_log.DebugFormat("[SCENE PRESENCE]: Updating MovementFlag for {0} with lack of {1}", Name, DCF);
                                MovementFlag -= ((byte)(uint)DCF);
                                update_movementflag = true;

                                /*
                                    if ((DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_FORWARD_NUDGE || DCF == Dir_ControlFlags.DIR_CONTROL_FLAG_BACKWARD_NUDGE)
                                    && ((MovementFlag & (byte)nudgehack) == nudgehack))
                                    {
                                        m_log.Debug("Removed Hack flag");
                                    }
                                */
                            }
                            else
                            {
                                bAllowUpdateMoveToPosition = true;
                            }
                        }

                        i++;
                    }

                    if (MovingToTarget)
                    {
                        // If the user has pressed a key then we want to cancel any move to target.
                        if (DCFlagKeyPressed)
                        {
                            ResetMoveToTarget();
                            update_movementflag = true;
                        }
                        else if (bAllowUpdateMoveToPosition)
                        {
                            if (HandleMoveToTargetUpdate(ref agent_control_v3))
                                update_movementflag = true;
                        }
                    }
                }

                // Cause the avatar to stop flying if it's colliding
                // with something with the down arrow pressed.

                // Only do this if we're flying
                if (PhysicsActor != null && PhysicsActor.Flying && !ForceFly)
                {
                    // Landing detection code

                    // Are the landing controls requirements filled?
                    bool controlland = (((flags & AgentManager.ControlFlags.AGENT_CONTROL_UP_NEG) != 0) ||
                                        ((flags & AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_NEG) != 0));

                    if (PhysicsActor.Flying && PhysicsActor.IsColliding && controlland)
                    {
                        // nesting this check because LengthSquared() is expensive and we don't 
                        // want to do it every step when flying.
                        if ((Velocity.LengthSquared() <= LAND_VELOCITYMAG_MAX))
                            StopFlying();
                    }
                }

                // If the agent update does move the avatar, then calculate the force ready for the velocity update,
                // which occurs later in the main scene loop
                if (update_movementflag || (update_rotation && DCFlagKeyPressed))
                {
//                    m_log.DebugFormat(
//                        "[SCENE PRESENCE]: In {0} adding velocity of {1} to {2}, umf = {3}, ur = {4}",
//                        m_scene.RegionInfo.RegionName, agent_control_v3, Name, update_movementflag, update_rotation);

                    AddNewMovement(agent_control_v3);
                }
//                else
//                {
//                    if (!update_movementflag)
//                    {
//                        m_log.DebugFormat(
//                            "[SCENE PRESENCE]: In {0} ignoring requested update of {1} for {2} as update_movementflag = false",
//                            m_scene.RegionInfo.RegionName, agent_control_v3, Name);
//                    }
//                }

//                if (update_movementflag && ParentID == 0)
//                    Animator.UpdateMovementAnimations();
            }

            m_scene.EventManager.TriggerOnClientMovement(this);
        }

        /// <summary>
        /// Calculate an update to move the presence to the set target.
        /// </summary>
        /// <remarks>
        /// This doesn't actually perform the movement.  Instead, it adds its vector to agent_control_v3.
        /// </remarks>
        /// <param value="agent_control_v3">Cumulative agent movement that this method will update.</param>
        /// <returns>True if movement has been updated in some way.  False otherwise.</returns>
        public bool HandleMoveToTargetUpdate(ref Vector3 agent_control_v3)
        {
//            m_log.DebugFormat("[SCENE PRESENCE]: Called HandleMoveToTargetUpdate() for {0}", Name);

            bool updated = false;

//            m_log.DebugFormat(
//                "[SCENE PRESENCE]: bAllowUpdateMoveToPosition {0}, m_moveToPositionInProgress {1}, m_autopilotMoving {2}",
//                allowUpdate, m_moveToPositionInProgress, m_autopilotMoving);

            double distanceToTarget = Util.GetDistanceTo(AbsolutePosition, MoveToPositionTarget);

//                        m_log.DebugFormat(
//                            "[SCENE PRESENCE]: Abs pos of {0} is {1}, target {2}, distance {3}",
//                            Name, AbsolutePosition, MoveToPositionTarget, distanceToTarget);

            // Check the error term of the current position in relation to the target position
            if (distanceToTarget <= 1)
            {
                // We are close enough to the target
                AbsolutePosition = MoveToPositionTarget;
                ResetMoveToTarget();
                updated = true;
            }
            else
            {
                try
                {
                    // move avatar in 3D at one meter/second towards target, in avatar coordinate frame.
                    // This movement vector gets added to the velocity through AddNewMovement().
                    // Theoretically we might need a more complex PID approach here if other
                    // unknown forces are acting on the avatar and we need to adaptively respond
                    // to such forces, but the following simple approach seems to works fine.
                    Vector3 LocalVectorToTarget3D =
                        (MoveToPositionTarget - AbsolutePosition) // vector from cur. pos to target in global coords
                        * Matrix4.CreateFromQuaternion(Quaternion.Inverse(m_bodyRot)); // change to avatar coords
                    // Ignore z component of vector
//                        Vector3 LocalVectorToTarget2D = new Vector3((float)(LocalVectorToTarget3D.X), (float)(LocalVectorToTarget3D.Y), 0f);
                    LocalVectorToTarget3D.Normalize();

                    // update avatar movement flags. the avatar coordinate system is as follows:
                    //
                    //                        +X (forward)
                    //
                    //                        ^
                    //                        |
                    //                        |
                    //                        |
                    //                        |
                    //     (left) +Y <--------o--------> -Y
                    //                       avatar
                    //                        |
                    //                        |
                    //                        |
                    //                        |
                    //                        v
                    //                        -X
                    //

                    // based on the above avatar coordinate system, classify the movement into
                    // one of left/right/back/forward.
                    if (LocalVectorToTarget3D.X < 0) //MoveBack
                    {
                        MovementFlag += (byte)(uint)Dir_ControlFlags.DIR_CONTROL_FLAG_BACK;
                        AgentControlFlags |= (uint)Dir_ControlFlags.DIR_CONTROL_FLAG_BACK;
                        updated = true;
                    }
                    else if (LocalVectorToTarget3D.X > 0) //Move Forward
                    {
                        MovementFlag += (byte)(uint)Dir_ControlFlags.DIR_CONTROL_FLAG_FORWARD;
                        AgentControlFlags |= (uint)Dir_ControlFlags.DIR_CONTROL_FLAG_FORWARD;
                        updated = true;
                    }

                    if (LocalVectorToTarget3D.Y > 0) //MoveLeft
                    {
                        MovementFlag += (byte)(uint)Dir_ControlFlags.DIR_CONTROL_FLAG_LEFT;
                        AgentControlFlags |= (uint)Dir_ControlFlags.DIR_CONTROL_FLAG_LEFT;
                        updated = true;
                    }
                    else if (LocalVectorToTarget3D.Y < 0) //MoveRight
                    {
                        MovementFlag += (byte)(uint)Dir_ControlFlags.DIR_CONTROL_FLAG_RIGHT;
                        AgentControlFlags |= (uint)Dir_ControlFlags.DIR_CONTROL_FLAG_RIGHT;
                        updated = true;
                    }

                    if (LocalVectorToTarget3D.Z > 0) //Up
                    {
                        // Don't set these flags for up or down - doing so will make the avatar crouch or
                        // keep trying to jump even if walking along level ground
                        //MovementFlag += (byte)(uint)Dir_ControlFlags.DIR_CONTROL_FLAG_UP;
                        //AgentControlFlags
                        //AgentControlFlags |= (uint)Dir_ControlFlags.DIR_CONTROL_FLAG_UP;
                        updated = true;
                    }
                    else if (LocalVectorToTarget3D.Z < 0) //Down
                    {
                        //MovementFlag += (byte)(uint)Dir_ControlFlags.DIR_CONTROL_FLAG_DOWN;
                        //AgentControlFlags |= (uint)Dir_ControlFlags.DIR_CONTROL_FLAG_DOWN;
                        updated = true;
                    }

//                        m_log.DebugFormat(
//                            "[SCENE PRESENCE]: HandleMoveToTargetUpdate adding {0} to move vector {1} for {2}",
//                            LocalVectorToTarget3D, agent_control_v3, Name);

                    agent_control_v3 += LocalVectorToTarget3D;
                }
                catch (Exception e)
                {
                    //Avoid system crash, can be slower but...
                    m_log.DebugFormat("Crash! {0}", e.ToString());
                }
            }

            return updated;
        }

        /// <summary>
        /// Move to the given target over time.
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="noFly">
        /// If true, then don't allow the avatar to fly to the target, even if it's up in the air.
        /// This is to allow movement to targets that are known to be on an elevated platform with a continuous path
        /// from start to finish.
        /// </param>
        /// <param name="landAtTarget">
        /// If true and the avatar starts flying during the move then land at the target.
        /// </param>
        public void MoveToTarget(Vector3 pos, bool noFly, bool landAtTarget)
        {
            if (SitGround)
                StandUp();

//            m_log.DebugFormat(
//                "[SCENE PRESENCE]: Avatar {0} received request to move to position {1} in {2}",
//                Name, pos, m_scene.RegionInfo.RegionName);

            if (pos.X < 0 || pos.X >= Constants.RegionSize
                || pos.Y < 0 || pos.Y >= Constants.RegionSize
                || pos.Z < 0)
                return;

//            Vector3 heightAdjust = new Vector3(0, 0, Appearance.AvatarHeight / 2);
//            pos += heightAdjust;
//
//            // Anti duck-walking measure
//            if (Math.Abs(pos.Z - AbsolutePosition.Z) < 0.2f)
//            {
////                m_log.DebugFormat("[SCENE PRESENCE]: Adjusting MoveToPosition from {0} to {1}", pos, AbsolutePosition);
//                pos.Z = AbsolutePosition.Z;
//            }

            float terrainHeight = (float)m_scene.Heightmap[(int)pos.X, (int)pos.Y];
            pos.Z = Math.Max(terrainHeight, pos.Z);

            // Fudge factor.  It appears that if one clicks "go here" on a piece of ground, the go here request is
            // always slightly higher than the actual terrain height.
            // FIXME: This constrains NPC movements as well, so should be somewhere else.
            if (pos.Z - terrainHeight < 0.2)
                pos.Z = terrainHeight;

            m_log.DebugFormat(
                "[SCENE PRESENCE]: Avatar {0} set move to target {1} (terrain height {2}) in {3}",
                Name, pos, terrainHeight, m_scene.RegionInfo.RegionName);

            if (noFly)
                PhysicsActor.Flying = false;
            else if (pos.Z > terrainHeight)
                PhysicsActor.Flying = true;

            LandAtTarget = landAtTarget;
            MovingToTarget = true;
            MoveToPositionTarget = pos;

            // Rotate presence around the z-axis to point in same direction as movement.
            // Ignore z component of vector
            Vector3 localVectorToTarget3D = pos - AbsolutePosition;
            Vector3 localVectorToTarget2D = new Vector3((float)(localVectorToTarget3D.X), (float)(localVectorToTarget3D.Y), 0f);

//            m_log.DebugFormat("[SCENE PRESENCE]: Local vector to target is {0}", localVectorToTarget2D);

            // Calculate the yaw.
            Vector3 angle = new Vector3(0, 0, (float)(Math.Atan2(localVectorToTarget2D.Y, localVectorToTarget2D.X)));

//            m_log.DebugFormat("[SCENE PRESENCE]: Angle is {0}", angle);

            Rotation = Quaternion.CreateFromEulers(angle);
//            m_log.DebugFormat("[SCENE PRESENCE]: Body rot for {0} set to {1}", Name, Rotation);
            
            Vector3 agent_control_v3 = new Vector3();
            HandleMoveToTargetUpdate(ref agent_control_v3);
            AddNewMovement(agent_control_v3);
        }

        /// <summary>
        /// Reset the move to target.
        /// </summary>
        public void ResetMoveToTarget()
        {
//            m_log.DebugFormat("[SCENE PRESENCE]: Resetting move to target for {0}", Name);

            MovingToTarget = false;
            MoveToPositionTarget = Vector3.Zero;

            // We need to reset the control flag as the ScenePresenceAnimator uses this to determine the correct
            // resting animation (e.g. hover or stand).  NPCs don't have a client that will quickly reset this flag.
            // However, the line is here rather than in the NPC module since it also appears necessary to stop a
            // viewer that uses "go here" from juddering on all subsequent avatar movements.
            AgentControlFlags = (uint)AgentManager.ControlFlags.NONE;
        }

        /// <summary>
        /// Perform the logic necessary to stand the avatar up.  This method also executes
        /// the stand animation.
        /// </summary>
        public void StandUp()
        {
            //            m_log.DebugFormat("[SCENE PRESENCE]: StandUp() for {0}", Name);

            SitGround = false;
            if (PhysicsActor == null)
                AddToPhysicalScene(false);

            if (m_sitPart != null)
            {
                TaskInventoryDictionary taskIDict = m_sitPart.TaskInventory;
                if (taskIDict != null)
                {
                    lock (taskIDict)
                    {
                        foreach (UUID taskID in taskIDict.Keys)
                        {
                            UnRegisterControlEventsToScript(LocalId, taskID);
                            taskIDict[taskID].PermsMask &= ~(
                                2048 | //PERMISSION_CONTROL_CAMERA
                                4); // PERMISSION_TAKE_CONTROLS
                        }
                    }
                }

                // Reset sit target.
                if (m_sitPart.SitTargetAvatar == UUID)
                    m_sitPart.SitTargetAvatar = UUID.Zero;

                m_pos += m_sitPart.GetWorldPosition() + new Vector3(0.0f, 0.0f, 2.0f * m_sitAvatarHeight);

                ControllingClient.SendClearFollowCamProperties(m_sitPart.ParentUUID);
                m_sitPart.ParentGroup.TriggerScriptChangedEvent(Changed.LINK);
            }


            ParentID = 0;
            SitPartID = 0;
            m_sitPart = null;

            SendAvatarDataToAllAgents();
            m_requestedSitTargetID = 0;

            Animator.TrySetMovementAnimation("STAND");
        }

        private SceneObjectPart FindNextAvailableSitTarget(UUID targetID)
        {
            SceneObjectPart targetPart = m_scene.GetSceneObjectPart(targetID);
            if (targetPart == null)
                return null;

            // If the primitive the player clicked on has a sit target and that sit target is not full, that sit target is used.
            // If the primitive the player clicked on has no sit target, and one or more other linked objects have sit targets that are not full, the sit target of the object with the lowest link number will be used.

            // Get our own copy of the part array, and sort into the order we want to test
            SceneObjectPart[] partArray = targetPart.ParentGroup.Parts;
            Array.Sort(partArray, delegate(SceneObjectPart p1, SceneObjectPart p2)
                       {
                           // we want the originally selected part first, then the rest in link order -- so make the selected part link num (-1)
                           int linkNum1 = p1==targetPart ? -1 : p1.LinkNum;
                           int linkNum2 = p2==targetPart ? -1 : p2.LinkNum;
                           return linkNum1 - linkNum2;
                       }
                );

            //look for prims with explicit sit targets that are available
            foreach (SceneObjectPart part in partArray)
            {
                // Is a sit target available?
               if(part.SitTargetPosition != Vector3.Zero && part.SitTargetAvatar == UUID.Zero)
                {
                    //switch the target to this prim
                    return part;
                }
            }

            // no explicit sit target found - use original target
            return targetPart;
        }

        private void SendSitResponse(UUID targetID, Vector3 offset, Quaternion pSitOrientation)
        {

            SceneObjectPart part = FindNextAvailableSitTarget(targetID);

            if (part == null)              
                return;

            Vector3 pos = new Vector3();
            Vector3 cameraEyeOffset = Vector3.Zero;
            Vector3 cameraAtOffset = Vector3.Zero;
            bool forceMouselook = false;
            m_requestedSitOffset = offset;
            m_requestedSitOrientation = pSitOrientation;

            // TODO: determine position to sit at based on scene geometry; don't trust offset from client
            // see http://wiki.secondlife.com/wiki/User:Andrew_Linden/Office_Hours/2007_11_06 for details on how LL does it

            // Is a sit target available?
            Vector3 avSitOffSet = part.SitTargetPosition;
            Quaternion avSitOrientation = part.SitTargetOrientation;
            UUID avOnTargetAlready = part.SitTargetAvatar;

            bool SitTargetUnOccupied = (avOnTargetAlready == UUID.Zero);
            bool SitTargetisSet = (avSitOffSet != Vector3.Zero);

//                m_log.DebugFormat("[SCENE PRESENCE]: {0} {1}", SitTargetisSet, SitTargetUnOccupied);

            if (PhysicsActor != null)
                m_sitAvatarHeight = m_physicsActor.Size.Z;

            bool canSit = false;
            pos = part.AbsolutePosition + offset;

            if (SitTargetisSet)
            {
                if (SitTargetUnOccupied)
                {
//                    m_log.DebugFormat(
//                        "[SCENE PRESENCE]: Sitting {0} on {1} {2} because sit target is set and unoccupied",
//                        Name, part.Name, part.LocalId);

                    part.SitTargetAvatar = UUID;
                    m_requestedSitOffset = new Vector3(avSitOffSet.X, avSitOffSet.Y, avSitOffSet.Z); 
                    m_requestedSitOrientation = avSitOrientation;
                    canSit = true;
                }
            }
            else
            {
                if (Util.GetDistanceTo(AbsolutePosition, pos) <= 10)
                {
//                    m_log.DebugFormat(
//                        "[SCENE PRESENCE]: Sitting {0} on {1} {2} because sit target is unset and within 10m",
//                        Name, part.Name, part.LocalId);

                    AbsolutePosition = pos + new Vector3(0.0f, 0.0f, m_sitAvatarHeight);
                    canSit = true;
                }
            }

            if (canSit)
            {
                if (PhysicsActor != null)
                {
                    // We can remove the physicsActor until they stand up.
                    RemoveFromPhysicalScene();
                }

                cameraAtOffset = part.GetCameraAtOffset();
                cameraEyeOffset = part.GetCameraEyeOffset();
                forceMouselook = part.GetForceMouselook();

                UUID objectParentUUID = part.ParentUUID;
                ControllingClient.SendSitResponse(
//                    targetID, m_requestedSitOffset, m_requestedSitOrientation, false, cameraAtOffset, cameraEyeOffset, forceMouselook);
                    objectParentUUID, m_requestedSitOffset, m_requestedSitOrientation, false, cameraAtOffset, cameraEyeOffset, forceMouselook);

                m_requestedSitTargetUUID = targetID;

                HandleAgentSit(ControllingClient, UUID);

                // Moved here to avoid a race with default sit anim
                // The script event needs to be raised after the default sit anim is set.
                part.ParentGroup.TriggerScriptChangedEvent(Changed.LINK);
            }
        }

        public void HandleAgentRequestSit(IClientAPI remoteClient, UUID agentID, UUID targetID, Vector3 offset)
        {
            if (ParentID != 0)
            {
                StandUp();
            }

//            if (!String.IsNullOrEmpty(sitAnimation))
//            {
//                m_nextSitAnimation = sitAnimation;
//            }
//            else
//            {
            m_nextSitAnimation = "SIT";
//            }

            //SceneObjectPart part = m_scene.GetSceneObjectPart(targetID);
            SceneObjectPart part = FindNextAvailableSitTarget(targetID);

            if (part != null)
            {
                if (!String.IsNullOrEmpty(part.SitAnimation))
                {
                    m_nextSitAnimation = part.SitAnimation;
                }

                m_requestedSitTargetID = part.LocalId;
                m_requestedSitTargetUUID = targetID;

//                m_log.DebugFormat("[SIT]: Client requested Sit Position: {0}", offset);

                // if we have a sit target don't look for one
                if (part.SitTargetPosition != Vector3.Zero)
                {
                    SendSitResponse(targetID, offset, Quaternion.Identity);
                    return;
                }

                if (m_scene.PhysicsScene.SupportsRayCast())
                {
                    m_requestedSitOffset = offset;
                    //m_scene.PhysicsScene.RaycastWorld(Vector3.Zero,Vector3.Zero, 0.01f,new RaycastCallback());
                    //SitRayCastAvatarPosition(part);
                    //return;
                }
            }
            else
            {
                m_log.Warn("Sit requested on unknown object: " + targetID.ToString());
            }

            SendSitResponse(targetID, offset, Quaternion.Identity);
        }

        //  find surface point near selected spot
        public void SitRayCastFindDirectContact(SceneObjectPart part)
        {
            Vector3 EndRayCastPosition = part.AbsolutePosition + m_requestedSitOffset;
            Vector3 StartRayCastPosition = AbsolutePosition;

            Vector3 direction = EndRayCastPosition - StartRayCastPosition;
            float distance = direction.Length();
            direction /= distance;
            m_scene.PhysicsScene.RaycastActor(part.PhysActor,StartRayCastPosition, direction, distance, SitRayCastFindDirectContactResponse);
        }

        public void SitRayCastFindDirectContactResponse(bool hitYN, Vector3 collisionPoint, uint localid, float pdistance, Vector3 normal)
        {
            SceneObjectPart part = FindNextAvailableSitTarget(m_requestedSitTargetUUID);
            if (part != null)
            {
                if (hitYN)
                {
                    if (localid == m_requestedSitTargetID)
                    {
                        SitRaycastFoundPath(part, collisionPoint, normal);
                        m_log.DebugFormat("[SIT]: Raycast Avatar Position succeeded at point: {0}, normal:{1}", collisionPoint, normal);
                    }
                    else
                    {
                        SitRayCastAvatarPositionCameraZ(part);
                    }
                }
                else
                {
                    SitRayCastAvatarPositionCameraZ(part);
                }
            }
            else
            {
                ControllingClient.SendAlertMessage("Sit position no longer exists");
                m_requestedSitTargetUUID = UUID.Zero;
                m_requestedSitTargetID = 0;
                m_requestedSitOffset = Vector3.Zero;
            }

        }

        // see if there is a clear path along position to requested position on part
        public void SitRayCastAvatarPosition(SceneObjectPart part)
        {
            Vector3 EndRayCastPosition = part.AbsolutePosition + m_requestedSitOffset;
            Vector3 StartRayCastPosition = AbsolutePosition;
            Vector3 direction = EndRayCastPosition - StartRayCastPosition;
            float distance = direction.Length();
            direction /= distance;
            m_scene.PhysicsScene.RaycastWorld(StartRayCastPosition, direction, distance, SitRayCastAvatarPositionResponse);
        }

        public void SitRayCastAvatarPositionResponse(bool hitYN, Vector3 collisionPoint, uint localid, float pdistance, Vector3 normal)
        {
            SceneObjectPart part =  FindNextAvailableSitTarget(m_requestedSitTargetUUID);
            if (part != null)
            {
                if (hitYN)
                {
                    if (localid == m_requestedSitTargetID)
                    {
                        SitRaycastFoundPath(part, collisionPoint, normal);
                        m_log.DebugFormat("[SIT]: Raycast Avatar Position succeeded at point: {0}, normal:{1}", collisionPoint, normal);
                    }
                    else
                    {
                        SitRayCastAvatarPositionCameraZ(part);
                    }
                }
                else
                {
                    SitRayCastAvatarPositionCameraZ(part);
                }
            }
            else
            {
                ControllingClient.SendAlertMessage("Sit position no longer exists");
                m_requestedSitTargetUUID = UUID.Zero;
                m_requestedSitTargetID = 0;
                m_requestedSitOffset = Vector3.Zero;
            }

        }

        // see if there is a clear path along position at camera height to requested position on part
        public void SitRayCastAvatarPositionCameraZ(SceneObjectPart part)
        {
            // Next, try to raycast from the camera Z position
            Vector3 EndRayCastPosition = part.AbsolutePosition + m_requestedSitOffset;
            Vector3 StartRayCastPosition = AbsolutePosition;
            StartRayCastPosition.Z = CameraPosition.Z;
            Vector3 direction = EndRayCastPosition - StartRayCastPosition;
            float distance = direction.Length();
            direction /= distance;
            m_scene.PhysicsScene.RaycastWorld(StartRayCastPosition, direction, distance, SitRayCastAvatarPositionCameraZResponse);
        }

        public void SitRayCastAvatarPositionCameraZResponse(bool hitYN, Vector3 collisionPoint, uint localid, float pdistance, Vector3 normal)
        {
            SceneObjectPart part = FindNextAvailableSitTarget(m_requestedSitTargetUUID);
            if (part != null)
            {
                if (hitYN)
                {
                    if (localid == m_requestedSitTargetID)
                    {
                        SitRaycastFoundPath(part, collisionPoint, normal);
                        m_log.DebugFormat("[SIT]: Raycast Avatar Position + CameraZ succeeded at point: {0}, normal:{1}", collisionPoint, normal);
                    }
                    else
                    {
                        SitRayCastCameraPosition(part);
                    }
                }
                else
                {
                    SitRayCastCameraPosition(part);
                }
            }
            else
            {
                ControllingClient.SendAlertMessage("Sit position no longer exists");
                m_requestedSitTargetUUID = UUID.Zero;
                m_requestedSitTargetID = 0;
                m_requestedSitOffset = Vector3.Zero;
            }

        }

        // see if there is a clear path from camera to requested position on part

        public void SitRayCastCameraPosition(SceneObjectPart part)
        {
            // Next, try to raycast from the camera position
            Vector3 EndRayCastPosition = part.AbsolutePosition + m_requestedSitOffset;
            Vector3 StartRayCastPosition = CameraPosition;
            Vector3 direction = EndRayCastPosition - StartRayCastPosition;
            float distance = direction.Length();
            direction /= distance;
            m_scene.PhysicsScene.RaycastWorld(StartRayCastPosition, direction, distance, SitRayCastCameraPositionResponse);
        }

        public void SitRayCastCameraPositionResponse(bool hitYN, Vector3 collisionPoint, uint localid, float pdistance, Vector3 normal)
        {
            SceneObjectPart part = FindNextAvailableSitTarget(m_requestedSitTargetUUID);
            if (part != null)
            {
                if (hitYN)
                {
                    if (localid == m_requestedSitTargetID)
                    {
                        SitRaycastFoundPath(part, collisionPoint, normal);
                        m_log.DebugFormat("[SIT]: Raycast Camera Position succeeded at point: {0}, normal:{1}", collisionPoint, normal);
                    }
                    else
                    {
                        //                        SitRayHorizontal(part);
                        ControllingClient.SendAlertMessage("Sit position not accessable.");
                        m_requestedSitTargetUUID = UUID.Zero;
                        m_requestedSitTargetID = 0;
                        m_requestedSitOffset = Vector3.Zero;

                    }
                }
                else
                {
                    //                    SitRayHorizontal(part);
                    ControllingClient.SendAlertMessage("Sit position not accessable.");
                    m_requestedSitTargetUUID = UUID.Zero;
                    m_requestedSitTargetID = 0;
                    m_requestedSitOffset = Vector3.Zero;
                }
            }
            else
            {
                ControllingClient.SendAlertMessage("Sit position no longer exists");
                m_requestedSitTargetUUID = UUID.Zero;
                m_requestedSitTargetID = 0;
                m_requestedSitOffset = Vector3.Zero;
            }
        }

/* not in use
        public void SitRayHorizontal(SceneObjectPart part)
        {
            // Next, try to raycast from the avatar position to fwd
            Vector3 EndRayCastPosition = part.AbsolutePosition + m_requestedSitOffset;
            Vector3 StartRayCastPosition = CameraPosition;
            Vector3 direction = EndRayCastPosition - StartRayCastPosition;
            float distance = direction.Length();
            direction /= distance;
            m_scene.PhysicsScene.RaycastWorld(StartRayCastPosition, direction, distance, SitRayCastHorizontalResponse);
        }

        public void SitRayCastHorizontalResponse(bool hitYN, Vector3 collisionPoint, uint localid, float pdistance, Vector3 normal)
        {
            SceneObjectPart part = FindNextAvailableSitTarget(m_requestedSitTargetUUID);
            if (part != null)
            {
                if (hitYN)
                {
                    if (localid == m_requestedSitTargetID)
                    {
                        SitRaycastFindEdge(part,collisionPoint, normal);
                        m_log.DebugFormat("[SIT]: Raycast Horizontal Position succeeded at point: {0}, normal:{1}", collisionPoint, normal);
                        // Next, try to raycast from the camera position
                        Vector3 EndRayCastPosition = part.AbsolutePosition + m_requestedSitOffset;
                        Vector3 StartRayCastPosition = CameraPosition;
                        Vector3 direction = Vector3.Normalize(EndRayCastPosition - StartRayCastPosition);
                        float distance = Vector3.Distance(EndRayCastPosition, StartRayCastPosition);
                        //m_scene.PhysicsScene.RaycastWorld(StartRayCastPosition, direction, distance, SitRayCastResponseAvatarPosition);
                    }
                    else
                    {
                        ControllingClient.SendAlertMessage("Sit position not accessable.");
                        m_requestedSitTargetUUID = UUID.Zero;
                        m_requestedSitTargetID = 0;
                        m_requestedSitOffset = Vector3.Zero;
                    }
                }
                else
                {
                    ControllingClient.SendAlertMessage("Sit position not accessable.");
                    m_requestedSitTargetUUID = UUID.Zero;
                    m_requestedSitTargetID = 0;
                    m_requestedSitOffset = Vector3.Zero;
                }
            }
            else
            {
                ControllingClient.SendAlertMessage("Sit position no longer exists");
                m_requestedSitTargetUUID = UUID.Zero;
                m_requestedSitTargetID = 0;
                m_requestedSitOffset = Vector3.Zero;
            }

        }
*/

        private void SitRaycastFoundPath(SceneObjectPart part, Vector3 collisionPoint, Vector3 collisionNormal)
        {
            // go to top of part, above nearest point


            Vector3 partscale = part.Scale * part.GetWorldRotation(); // part AABB
            if (partscale.Z <0 )
                partscale.Z = -partscale.Z;


            if (part.GetPrimType() == PrimType.SPHERE)
            {
                Vector3 contactpos = new Vector3(0,0,partscale.Z);
                SendSitResponse(m_requestedSitTargetUUID, contactpos, Quaternion.Identity);
                return;
            }
            
            
            collisionPoint.Z += partscale.Z;
            collisionNormal.Z += 0.5f;

            Vector3 EndRayCastPosition = part.AbsolutePosition;
            Vector3 StartRayCastPosition =  collisionPoint;
            Vector3 direction = EndRayCastPosition - StartRayCastPosition;
            float distance = direction.Length();
            direction /= distance;
            m_scene.PhysicsScene.RaycastWorld(StartRayCastPosition, direction, distance, SitRayCastfindTop);
        }

        public void SitRayCastfindTop(bool hitYN, Vector3 collisionPoint, uint localid, float pdistance, Vector3 normal)
        {
            SceneObjectPart part = FindNextAvailableSitTarget(m_requestedSitTargetUUID);
            if (part != null)
            {
                if (hitYN)
                {
                    if (localid == m_requestedSitTargetID)
                    {
                        SitRayCastfoundTop(part, collisionPoint, normal);
                        m_log.DebugFormat("[SIT]: Raycast Camera Position succeeded at point: {0}, normal:{1}", collisionPoint, normal);
                    }
                    else
                    {
                        //                        SitRayHorizontal(part);
                        ControllingClient.SendAlertMessage("Sit position not accessable.");
                        m_requestedSitTargetUUID = UUID.Zero;
                        m_requestedSitTargetID = 0;
                        m_requestedSitOffset = Vector3.Zero;
                    }
                }
                else
                {
                    //                    SitRayHorizontal(part);
                    ControllingClient.SendAlertMessage("Sit position not accessable.");
                    m_requestedSitTargetUUID = UUID.Zero;
                    m_requestedSitTargetID = 0;
                    m_requestedSitOffset = Vector3.Zero;
                }
            }
            else
            {
                ControllingClient.SendAlertMessage("Sit position no longer exists");
                m_requestedSitTargetUUID = UUID.Zero;
                m_requestedSitTargetID = 0;
                m_requestedSitOffset = Vector3.Zero;
            }
        }

        private void SitRayCastfoundTop(SceneObjectPart part, Vector3 collisionPoint, Vector3 collisionNormal)
        {
            Vector3 partpos = part.AbsolutePosition;
            Vector3 contactpos = collisionPoint - partpos; // contact relative to part
            Quaternion partrot = part.GetWorldRotation();

            contactpos *= Quaternion.Inverse(partrot); // contact in part coords
//            collisionNormal *= Quaternion.Inverse(partrot);

            Quaternion rot = Vector3.RotationBetween(contactpos, new Vector3(1, 0, 0)); // angle with part z axis
            rot = Quaternion.CreateFromAxisAngle(contactpos,0);
            Vector3 avasize = new Vector3(0.37f, 0.37f, m_appearance.AvatarHeight * 0.5f);
            avasize *= rot;
//            contactpos.Z += avasize.Z + 0.15f;

            SendSitResponse(m_requestedSitTargetUUID, contactpos, rot);        
        }

        public void HandleAgentSit(IClientAPI remoteClient, UUID agentID)
        {
            if (!String.IsNullOrEmpty(m_nextSitAnimation))
            {
                HandleAgentSit(remoteClient, agentID, m_nextSitAnimation);
            }
            else
            {
                HandleAgentSit(remoteClient, agentID, "SIT");
            }
        }

        public void HandleAgentSit(IClientAPI remoteClient, UUID agentID, string sitAnimation)
        {
            SceneObjectPart part = m_scene.GetSceneObjectPart(m_requestedSitTargetID);

            if (part != null)
            {
                if (part.SitTargetAvatar == agentID)
                {
                    if (part.CreationDate > 1320537600) // 06/11/2011 0:0:0
                        m_pos = m_requestedSitOffset + SIT_TARGET_ADJUSTMENT;
                    else
                        m_pos = m_requestedSitOffset + OLD_SIT_TARGET_ADJUSTMENT;
                }
                else
                {
                    m_pos -= part.AbsolutePosition;
                    part.SitTargetAvatar = agentID;

//                        m_log.DebugFormat(
//                            "[SCENE PRESENCE]: Sitting {0} at position {1} ({2} + {3}) on part {4} {5} without sit target",
//                            Name, part.AbsolutePosition, m_pos, ParentPosition, part.Name, part.LocalId);
                }
                Rotation = m_requestedSitOrientation;
                m_sitPart = part;
            }
            else
            {
                m_sitPart = null;
                return;
            }

            SitPartID = m_requestedSitTargetID;

            if(part.ParentID == 0)
                ParentID = SitPartID;
            else
                ParentID = part.ParentID;

            Velocity = Vector3.Zero;
            RemoveFromPhysicalScene();

            Animator.TrySetMovementAnimation(sitAnimation);
            SendAvatarDataToAllAgents();
        }

        public void HandleAgentSitOnGround()
        {
//            m_updateCount = 0;  // Kill animation update burst so that the SIT_G.. will stick.
            Animator.TrySetMovementAnimation("SIT_GROUND_CONSTRAINED");
            SitGround = true;
            RemoveFromPhysicalScene();
        }

        /// <summary>
        /// Event handler for the 'Always run' setting on the client
        /// Tells the physics plugin to increase speed of movement.
        /// </summary>
        public void HandleSetAlwaysRun(IClientAPI remoteClient, bool pSetAlwaysRun)
        {
            SetAlwaysRun = pSetAlwaysRun;
        }

        public void HandleStartAnim(IClientAPI remoteClient, UUID animID)
        {
            Animator.AddAnimation(animID, UUID.Zero,true);
        }

        public void HandleStartAnim(IClientAPI remoteClient, UUID animID, bool sendPack)
        {
            Animator.AddAnimation(animID, UUID.Zero,sendPack);
        }

        public void HandleStopAnim(IClientAPI remoteClient, UUID animID)
        {
            Animator.RemoveAnimation(animID,true);
        }

        public void HandleStopAnim(IClientAPI remoteClient, UUID animID, bool SendPack)
        {
            Animator.RemoveAnimation(animID, SendPack);
        }

        /// <summary>
        /// Rotate the avatar to the given rotation and apply a movement in the given relative vector
        /// </summary>
        /// <param name="vec">The vector in which to move.  This is relative to the rotation argument</param>
        public void AddNewMovement(Vector3 vec)
        {
            Vector3 direc = vec * Rotation;
            direc.Normalize();

            if (PhysicsActor.Flying != m_flyingOld)                // add for fly velocity control
            {
                m_flyingOld = PhysicsActor.Flying;                 // add for fly velocity control
                if (!PhysicsActor.Flying)
                    m_wasFlying = true;      // add for fly velocity control
            }

            if (PhysicsActor.IsColliding == true)
                m_wasFlying = false;        // add for fly velocity control

            if ((vec.Z == 0f) && !PhysicsActor.Flying)
                direc.Z = 0f; // Prevent camera WASD up.

            direc *= 0.03f * 128f * SpeedModifier;

            if (PhysicsActor != null)
            {
                if (PhysicsActor.Flying)
                {
                    direc *= 4.0f;
                    //bool controlland = (((m_AgentControlFlags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_UP_NEG) != 0) || ((m_AgentControlFlags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_NEG) != 0));
                    //if (controlland)
                    //    m_log.Info("[AGENT]: landCommand");
                    //if (PhysicsActor.IsColliding)
                    //    m_log.Info("[AGENT]: colliding");
                    //if (PhysicsActor.Flying && PhysicsActor.IsColliding && controlland)
                    //{
                    //    StopFlying();
                    //    m_log.Info("[AGENT]: Stop Flying");
                    //}
                }
                if (Animator.Falling && m_wasFlying)    // if falling from flying, disable motion add
                {
                    direc *= 0.0f;
                }
                else if (!PhysicsActor.Flying && PhysicsActor.IsColliding)
                {
                    if (direc.Z > 2.0f)
                    {
                        direc.Z *= 3.0f;

                        // TODO: PreJump and jump happen too quickly.  Many times prejump gets ignored.
                        Animator.TrySetMovementAnimation("PREJUMP");
                        Animator.TrySetMovementAnimation("JUMP");
                    }
                }
            }

            // TODO: Add the force instead of only setting it to support multiple forces per frame?
            m_forceToApply = direc;
        }

        #endregion

        #region Overridden Methods

        public override void Update()
        {
            const float ROTATION_TOLERANCE = 0.01f;
            const float VELOCITY_TOLERANCE = 0.001f;
            const float POSITION_TOLERANCE = 0.05f;

            if (IsChildAgent == false && ParentID == 0) // also don't send for sitted avas
            {
//                PhysicsActor actor = m_physicsActor;

                // NOTE: Velocity is not the same as m_velocity. Velocity will attempt to
                // grab the latest PhysicsActor velocity, whereas m_velocity is often
                // storing a requested force instead of an actual traveling velocity

                // Throw away duplicate or insignificant updates
                if (!Rotation.ApproxEquals(m_lastRotation, ROTATION_TOLERANCE) ||
                    !Velocity.ApproxEquals(m_lastVelocity, VELOCITY_TOLERANCE) ||
                    !m_pos.ApproxEquals(m_lastPosition, POSITION_TOLERANCE))
                {
                    SendTerseUpdateToAllClients();

                    // Update the "last" values
                    m_lastPosition = m_pos;
                    m_lastRotation = Rotation;
                    m_lastVelocity = Velocity;
                }

                // followed suggestion from mic bowman. reversed the two lines below.
                if (ParentID == 0 && PhysicsActor != null) // || ParentID != 0) // Check that we have a physics actor or we're sitting on something
                    CheckForBorderCrossing();

                CheckForSignificantMovement(); // sends update to the modules.
            }
        }

        #endregion

        #region Update Client(s)


        /// <summary>
        /// Sends a location update to the client connected to this scenePresence
        /// </summary>
        /// <param name="remoteClient"></param>
        public void SendTerseUpdateToClient(IClientAPI remoteClient)
        {
            // If the client is inactive, it's getting its updates from another
            // server.
            if (remoteClient.IsActive)
            {
                //m_log.DebugFormat("[SCENE PRESENCE]: " + Name + " sending TerseUpdate to " + remoteClient.Name + " : Pos={0} Rot={1} Vel={2}", m_pos, Rotation, m_velocity);

                remoteClient.SendEntityUpdate(
                    this,
                    PrimUpdateFlags.Position | PrimUpdateFlags.Rotation | PrimUpdateFlags.Velocity
                    | PrimUpdateFlags.Acceleration | PrimUpdateFlags.AngularVelocity);

                m_scene.StatsReporter.AddAgentUpdates(1);
            }
        }


        // vars to support reduced update frequency when velocity is unchanged
        private Vector3 lastVelocitySentToAllClients = Vector3.Zero;
        private Vector3 lastPositionSentToAllClients = Vector3.Zero;
        private Quaternion lastRotationSentToAllClients = Quaternion.Identity;
        private int lastTerseUpdateToAllClientsTick = Util.EnvironmentTickCount();

        /// <summary>
        /// Send a location/velocity/accelleration update to all agents in scene
        /// </summary>
        public void SendTerseUpdateToAllClients()
        {
        // let's try another way

        Vector3 curvel = Velocity;
        Vector3 curpos = OffsetPosition;
        Quaternion currot = Rotation;
        int curtick = Util.EnvironmentTickCount();

        if ( // Math.Abs(curpos.X - lastPositionSentToAllClients.X) < 5f // didn't moved a lot ?
            // && Math.Abs(curpos.Y - lastPositionSentToAllClients.Y) < 5f
            //&& Math.Abs(curpos.Z - lastPositionSentToAllClients.Z) < 5f
            //&&
            Math.Abs(curvel.X - lastVelocitySentToAllClients.X) < 0.001f // not about change move ?
            && Math.Abs(curvel.Y - lastVelocitySentToAllClients.Y) < 0.001f
            && Math.Abs(curvel.Z - lastVelocitySentToAllClients.Z) < 0.01f
            && Math.Abs(currot.X - lastRotationSentToAllClients.X) < 0.001f // not about change rotation ?
            && Math.Abs(currot.Y - lastRotationSentToAllClients.Y) < 0.001f 
            && Math.Abs(currot.Z - lastRotationSentToAllClients.Z) < 0.001f 
            && Math.Abs(currot.W - lastRotationSentToAllClients.W) < 0.001f
            //&& curtick - lastTerseUpdateToAllClientsTick < 200 // not long ago ?
            )
            {
            return;
            }

// ok send update
        lastVelocitySentToAllClients = curvel;
        lastTerseUpdateToAllClientsTick = curtick;
        lastPositionSentToAllClients = curpos;
        lastRotationSentToAllClients = currot;

        m_scene.ForEachClient(SendTerseUpdateToClient);
        }
        public void SendCoarseLocations(List<Vector3> coarseLocations, List<UUID> avatarUUIDs)
        {
            SendCourseLocationsMethod d = m_sendCourseLocationsMethod;
            if (d != null)
            {
                d.Invoke(m_scene.RegionInfo.originRegionID, this, coarseLocations, avatarUUIDs);
            }
        }

        public void SetSendCourseLocationMethod(SendCourseLocationsMethod d)
        {
            if (d != null)
                m_sendCourseLocationsMethod = d;
        }

        public void SendCoarseLocationsDefault(UUID sceneId, ScenePresence p, List<Vector3> coarseLocations, List<UUID> avatarUUIDs)
        {
            ControllingClient.SendCoarseLocationUpdate(avatarUUIDs, coarseLocations);
        }

        public void SendInitialDataToMe()
        {
            // Send all scene object to the new client
            Util.FireAndForget(delegate
            {
                // we created a new ScenePresence (a new child agent) in a fresh region.
                // Request info about all the (root) agents in this region
                // Note: This won't send data *to* other clients in that region (children don't send)
                SendOtherAgentsAvatarDataToMe();
                SendOtherAgentsAppearanceToMe();

                EntityBase[] entities = Scene.Entities.GetEntities();
                foreach(EntityBase e in entities)
                {
                    if (e != null && e is SceneObjectGroup)
                        ((SceneObjectGroup)e).SendFullUpdateToClient(ControllingClient);
                }
            });
        }

        /// <summary>
        /// Do everything required once a client completes its movement into a region and becomes
        /// a root agent.
        /// </summary>
        private void ValidateAndSendAppearanceAndAgentData()
        {
            //m_log.DebugFormat("[SCENE PRESENCE] SendInitialData: {0} ({1})", Name, UUID);
            // Moved this into CompleteMovement to ensure that Appearance is initialized before
            // the inventory arrives
            // m_scene.GetAvatarAppearance(ControllingClient, out Appearance);

            bool cachedappearance = false;

            // We have an appearance but we may not have the baked textures. Check the asset cache 
            // to see if all the baked textures are already here. 
            if (m_scene.AvatarFactory != null)
                cachedappearance = m_scene.AvatarFactory.ValidateBakedTextureCache(this);
            
            // If we aren't using a cached appearance, then clear out the baked textures
            if (!cachedappearance)
            {
                Appearance.ResetAppearance();
                if (m_scene.AvatarFactory != null)
                    m_scene.AvatarFactory.QueueAppearanceSave(UUID);
            }
            
            // This agent just became root. We are going to tell everyone about it. The process of
            // getting other avatars information was initiated elsewhere immediately after the child circuit connected... don't do it
            // again here... this comes after the cached appearance check because the avatars
            // appearance goes into the avatar update packet
            SendAvatarDataToAllAgents();
            SendAppearanceToAgent(this);

            // If we are using the the cached appearance then send it out to everyone
            if (cachedappearance)
            {
                m_log.DebugFormat("[SCENEPRESENCE]: baked textures are in the cache for {0}", Name);

                // If the avatars baked textures are all in the cache, then we have a 
                // complete appearance... send it out, if not, then we'll send it when
                // the avatar finishes updating its appearance
                SendAppearanceToAllOtherAgents();
            }
        }

        /// <summary>
        /// Send this agent's avatar data to all other root and child agents in the scene
        /// This agent must be root. This avatar will receive its own update. 
        /// </summary>
        public void SendAvatarDataToAllAgents()
        {
            //m_log.DebugFormat("[SCENE PRESENCE] SendAvatarDataToAllAgents: {0} ({1})", Name, UUID);
            // only send update from root agents to other clients; children are only "listening posts"
            if (IsChildAgent)
            {
                m_log.Warn("[SCENE PRESENCE]: Attempt to send avatar data from a child agent");
                return;
            }

            int count = 0;
            m_scene.ForEachScenePresence(delegate(ScenePresence scenePresence)
                                         {
                                             SendAvatarDataToAgent(scenePresence);
                                             count++;
                                         });

            m_scene.StatsReporter.AddAgentUpdates(count);
        }

        /// <summary>
        /// Send avatar data for all other root agents to this agent, this agent
        /// can be either a child or root
        /// </summary>
        public void SendOtherAgentsAvatarDataToMe()
        {
            int count = 0;
            m_scene.ForEachRootScenePresence(delegate(ScenePresence scenePresence)
                        {
                            // only send information about other root agents
                            if (scenePresence.UUID == UUID)
                                return;
                                             
                            scenePresence.SendAvatarDataToAgent(this);
                            count++;
                        });

            m_scene.StatsReporter.AddAgentUpdates(count);
        }

        /// <summary>
        /// Send avatar data to an agent.
        /// </summary>
        /// <param name="avatar"></param>
        public void SendAvatarDataToAgent(ScenePresence avatar)
        {
            //m_log.DebugFormat("[SCENE PRESENCE] SendAvatarDataToAgent from {0} ({1}) to {2} ({3})", Name, UUID, avatar.Name, avatar.UUID);

            avatar.ControllingClient.SendAvatarDataImmediate(this);
            if (Animator != null)
                Animator.SendAnimPackToClient(avatar.ControllingClient);
        }

        /// <summary>
        /// Send this agent's appearance to all other root and child agents in the scene
        /// This agent must be root.
        /// </summary>
        public void SendAppearanceToAllOtherAgents()
        {
//            m_log.DebugFormat("[SCENE PRESENCE] SendAppearanceToAllOtherAgents: {0} {1}", Name, UUID);

            // only send update from root agents to other clients; children are only "listening posts"
            if (IsChildAgent)
            {
                m_log.Warn("[SCENE PRESENCE]: Attempt to send avatar data from a child agent");
                return;
            }
            
            int count = 0;
            m_scene.ForEachScenePresence(delegate(ScenePresence scenePresence)
                        {
                            // only send information to other root agents
                            if (scenePresence.UUID == UUID)
                                return;

                            SendAppearanceToAgent(scenePresence);
                            count++;
                        });

            m_scene.StatsReporter.AddAgentUpdates(count);
        }

        /// <summary>
        /// Send appearance from all other root agents to this agent. this agent
        /// can be either root or child
        /// </summary>
        public void SendOtherAgentsAppearanceToMe()
        {
//            m_log.DebugFormat("[SCENE PRESENCE] SendOtherAgentsAppearanceToMe: {0} {1}", Name, UUID);

            int count = 0;
            m_scene.ForEachRootScenePresence(delegate(ScenePresence scenePresence)
                        {
                            // only send information about other root agents
                            if (scenePresence.UUID == UUID)
                                return;
                                             
                            scenePresence.SendAppearanceToAgent(this);
                            count++;
                        });

            m_scene.StatsReporter.AddAgentUpdates(count);
        }

        /// <summary>
        /// Send appearance data to an agent.
        /// </summary>
        /// <param name="avatar"></param>
        public void SendAppearanceToAgent(ScenePresence avatar)
        {
//            m_log.DebugFormat(
//                "[SCENE PRESENCE] Send appearance from {0} {1} to {2} {3}", Name, m_uuid, avatar.Name, avatar.UUID);

            avatar.ControllingClient.SendAppearance(
                UUID, Appearance.VisualParams, Appearance.Texture.GetBytes());
        }

        #endregion

        #region Significant Movement Method

        /// <summary>
        /// This checks for a significant movement and sends a courselocationchange update
        /// </summary>
        protected void CheckForSignificantMovement()
        {
            if (Util.GetDistanceTo(AbsolutePosition, posLastSignificantMove) > SIGNIFICANT_MOVEMENT)
            {
                posLastSignificantMove = AbsolutePosition;
                m_scene.EventManager.TriggerSignificantClientMovement(this);
            }

            // Minimum Draw distance is 64 meters, the Radius of the draw distance sphere is 32m
            if (Util.GetDistanceTo(AbsolutePosition, m_lastChildAgentUpdatePosition) >= Scene.ChildReprioritizationDistance ||
                Util.GetDistanceTo(CameraPosition, m_lastChildAgentUpdateCamPosition) >= Scene.ChildReprioritizationDistance)
            {
                m_lastChildAgentUpdatePosition = AbsolutePosition;
                m_lastChildAgentUpdateCamPosition = CameraPosition;

                ChildAgentDataUpdate cadu = new ChildAgentDataUpdate();
                cadu.ActiveGroupID = UUID.Zero.Guid;
                cadu.AgentID = UUID.Guid;
                cadu.alwaysrun = SetAlwaysRun;
                cadu.AVHeight = Appearance.AvatarHeight;
                cadu.cameraPosition = CameraPosition;
                cadu.drawdistance = DrawDistance;
                cadu.GroupAccess = 0;
                cadu.Position = AbsolutePosition;
                cadu.regionHandle = RegionHandle;

                // Throttles 
                float multiplier = 1;
                int childRegions = KnownRegionCount;
                if (childRegions != 0)
                    multiplier = 1f / childRegions;

                // Minimum throttle for a child region is 1/4 of the root region throttle
                if (multiplier <= 0.25f)
                    multiplier = 0.25f;

                cadu.throttles = ControllingClient.GetThrottlesPacked(multiplier);
                cadu.Velocity = Velocity;

                AgentPosition agentpos = new AgentPosition();
                agentpos.CopyFrom(cadu);

                m_scene.SendOutChildAgentUpdates(agentpos, this);
            }
        }

        #endregion

        #region Border Crossing Methods

        /// <summary>
        /// Starts the process of moving an avatar into another region if they are crossing the border.
        /// </summary>
        /// <remarks>
        /// Also removes the avatar from the physical scene if transit has started.
        /// </remarks>
        protected void CheckForBorderCrossing()
        {
            if (IsChildAgent)
                return;

            Vector3 PredictedPos = AbsolutePosition;
            Vector3 vel = Velocity;

            //            float timeStep = 0.1f;
            float timeStep;

            if (IsInTransit)
            {
                // We must remove the agent from the physical scene if it has been placed in transit.  If we don't,
                // then this method continues to be called from ScenePresence.Update() until the handover of the client between
                // regions is completed.  Since this handover can take more than 1000ms (due to the 1000ms
                // event queue polling response from the server), this results in the avatar pausing on the border
                // for the handover period.
                RemoveFromPhysicalScene();

                // This constant has been inferred from experimentation
                // I'm not sure what this value should be, so I tried a few values.
                timeStep = m_scene.SimulationFrameTime * 0.001f * 0.25f;
                PredictedPos.X += (vel.X * timeStep);
                PredictedPos.Y += (vel.Y * timeStep);
                PredictedPos.Z += (vel.Z * timeStep);
                m_pos = PredictedPos;
                return;
            }

            bool neighbor = false;
            timeStep = m_scene.SimulationFrameTime * 0.001f;

            PredictedPos.X += (vel.X * timeStep);
            PredictedPos.Y += (vel.Y * timeStep);
            PredictedPos.Z += (vel.Z * timeStep);

            // Checks if where it's headed exists a region

            bool needsTransit = false;

            if (m_scene.RegionInfo.CombinedRegionHandle != 0)
            {
                needsTransit = true;
                neighbor = true;
            }

            else if (m_scene.TestBorderCross(PredictedPos, Cardinals.W))
            {
                if (m_scene.TestBorderCross(PredictedPos, Cardinals.S))
                {
                    needsTransit = true;
                    neighbor = m_scene.HaveNeighbor(PredictedPos);
                }
                else if (m_scene.TestBorderCross(PredictedPos, Cardinals.N))
                {
                    needsTransit = true;
                    neighbor = m_scene.HaveNeighbor(PredictedPos);
                }
                else
                {
                    needsTransit = true;
                    neighbor = m_scene.HaveNeighbor(PredictedPos);
                }
            }
            else if (m_scene.TestBorderCross(PredictedPos, Cardinals.E))
            {
                if (m_scene.TestBorderCross(PredictedPos, Cardinals.S))
                {
                    needsTransit = true;
                    neighbor = m_scene.HaveNeighbor(PredictedPos);
                }
                else if (m_scene.TestBorderCross(PredictedPos, Cardinals.N))
                {
                    needsTransit = true;
                    neighbor = m_scene.HaveNeighbor(PredictedPos);
                }
                else
                {
                    needsTransit = true;
                    neighbor = m_scene.HaveNeighbor(PredictedPos);
                }
            }
            else if (m_scene.TestBorderCross(PredictedPos, Cardinals.S))
            {
                needsTransit = true;
                neighbor = m_scene.HaveNeighbor(PredictedPos);
            }
            else if (m_scene.TestBorderCross(PredictedPos, Cardinals.N))
            {
                needsTransit = true;
                neighbor = m_scene.HaveNeighbor(PredictedPos);
            }

            if (needsTransit)
            {
                // try check and try crossing
                if (neighbor)
                {
                    if (CrossToNewRegion())
                        return;
                }

                // Makes sure avatar does not end up outside region
                if (m_requestedSitTargetUUID == UUID.Zero)
                {
                    bool isFlying = PhysicsActor.Flying;
                    RemoveFromPhysicalScene();

                    PredictedPos = AbsolutePosition;
                    if (PredictedPos.X - 0.5 < 0)
                        PredictedPos.X = 0.5f;
                    else if (PredictedPos.X + 0.5f > Scene.RegionInfo.RegionSizeX)
                        PredictedPos.X = Scene.RegionInfo.RegionSizeX - 0.5f;
                    if (PredictedPos.Y -0.5f < 0)
                        PredictedPos.Y = 0.5f;
                    else if (PredictedPos.Y + 0.5f > Scene.RegionInfo.RegionSizeY)
                        PredictedPos.Y = Scene.RegionInfo.RegionSizeY - 0.5f;
                    Velocity = Vector3.Zero;
                    AbsolutePosition = PredictedPos;

                    //                            m_log.DebugFormat("[SCENE PRESENCE]: Prevented flyoff for {0} at {1}", Name, AbsolutePosition);

                    AddToPhysicalScene(isFlying);
                }
            }
        }

        /// <summary>
        /// Moves the agent outside the region bounds
        /// Tells neighbor region that we're crossing to it
        /// If the neighbor accepts, remove the agent's viewable avatar from this scene
        /// set them to a child agent.
        /// </summary>
        protected bool CrossToNewRegion()
        {
            try
            {
                return m_scene.CrossAgentToNewRegion(this, PhysicsActor.Flying);
            }
            catch
            {
                return m_scene.CrossAgentToNewRegion(this, false);
            }
        }

        public void RestoreInCurrentScene()
        {
            AddToPhysicalScene(false); // not exactly false
        }

        public void Reset()
        {
//            m_log.DebugFormat("[SCENE PRESENCE]: Resetting {0} in {1}", Name, Scene.RegionInfo.RegionName);

            // Put the child agent back at the center
            AbsolutePosition 
                = new Vector3(((float)Constants.RegionSize * 0.5f), ((float)Constants.RegionSize * 0.5f), 70);

            Animator.ResetAnimations();
        }

        /// <summary>
        /// Computes which child agents to close when the scene presence moves to another region.
        /// Removes those regions from m_knownRegions.
        /// </summary>
        /// <param name="newRegionX">The new region's x on the map</param>
        /// <param name="newRegionY">The new region's y on the map</param>
        /// <returns></returns>
        public void CloseChildAgents(uint newRegionX, uint newRegionY)
        {
            List<ulong> byebyeRegions = new List<ulong>();
            List<ulong> knownRegions = KnownRegionHandles;
            m_log.DebugFormat(
                "[SCENE PRESENCE]: Closing child agents. Checking {0} regions in {1}", 
                knownRegions.Count, Scene.RegionInfo.RegionName);
            //DumpKnownRegions();

            foreach (ulong handle in knownRegions)
            {
                    // Don't close the agent on this region yet
                    if (handle != Scene.RegionInfo.RegionHandle)
                    {
                        uint x, y;
                        Utils.LongToUInts(handle, out x, out y);
                        x = x / Constants.RegionSize;
                        y = y / Constants.RegionSize;

                        //m_log.Debug("---> x: " + x + "; newx:" + newRegionX + "; Abs:" + (int)Math.Abs((int)(x - newRegionX)));
                        //m_log.Debug("---> y: " + y + "; newy:" + newRegionY + "; Abs:" + (int)Math.Abs((int)(y - newRegionY)));
                        if (Util.IsOutsideView(DrawDistance, x, newRegionX, y, newRegionY))
                        {
                            byebyeRegions.Add(handle);
                        }
                    }
                }
            
            if (byebyeRegions.Count > 0)
            {
                m_log.Debug("[SCENE PRESENCE]: Closing " + byebyeRegions.Count + " child agents");
                m_scene.SceneGridService.SendCloseChildAgentConnections(ControllingClient.AgentId, byebyeRegions);
            }
            
            foreach (ulong handle in byebyeRegions)
            {
                RemoveNeighbourRegion(handle);
            }
        }

        #endregion

        /// <summary>
        /// This allows the Sim owner the abiility to kick users from their sim currently.
        /// It tells the client that the agent has permission to do so.
        /// </summary>
        public void GrantGodlikePowers(UUID agentID, UUID sessionID, UUID token, bool godStatus)
        {
            if (godStatus)
            {
                // For now, assign god level 200 to anyone
                // who is granted god powers, but has no god level set.
                //
                UserAccount account = m_scene.UserAccountService.GetUserAccount(m_scene.RegionInfo.ScopeID, agentID);
                if (account != null)
                {
                    if (account.UserLevel > 0)
                        GodLevel = account.UserLevel;
                    else
                        GodLevel = 200;
                }
            }
            else
            {
                GodLevel = 0;
            }

            ControllingClient.SendAdminResponse(token, (uint)GodLevel);
        }

        #region Child Agent Updates

        public void ChildAgentDataUpdate(AgentData cAgentData)
        {
            //m_log.Debug("   >>> ChildAgentDataUpdate <<< " + Scene.RegionInfo.RegionName);
            if (!IsChildAgent)
                return;

            CopyFrom(cAgentData);
        }

        /// <summary>
        /// This updates important decision making data about a child agent
        /// The main purpose is to figure out what objects to send to a child agent that's in a neighboring region
        /// </summary>
        public void ChildAgentDataUpdate(AgentPosition cAgentData, uint tRegionX, uint tRegionY, uint rRegionX, uint rRegionY)
        {
            if (!IsChildAgent)
                return;

            //m_log.Debug("   >>> ChildAgentPositionUpdate <<< " + rRegionX + "-" + rRegionY);
            int shiftx = ((int)rRegionX - (int)tRegionX) * (int)Constants.RegionSize;
            int shifty = ((int)rRegionY - (int)tRegionY) * (int)Constants.RegionSize;

            Vector3 offset = new Vector3(shiftx, shifty, 0f);

            // When we get to the point of re-computing neighbors everytime this
            // changes, then start using the agent's drawdistance rather than the 
            // region's draw distance.
            // DrawDistance = cAgentData.Far;
            DrawDistance = Scene.DefaultDrawDistance;
            
            if (cAgentData.Position != new Vector3(-1f, -1f, -1f)) // UGH!!
                m_pos = cAgentData.Position + offset;

            if (Vector3.Distance(AbsolutePosition, posLastSignificantMove) >= Scene.ChildReprioritizationDistance)
            {
                posLastSignificantMove = AbsolutePosition;
                ReprioritizeUpdates();
            }

            CameraPosition = cAgentData.Center + offset;

            //SetHeight(cAgentData.AVHeight);

            if ((cAgentData.Throttles != null) && cAgentData.Throttles.Length > 0)
                ControllingClient.SetChildAgentThrottle(cAgentData.Throttles);

            //cAgentData.AVHeight;
            RegionHandle = cAgentData.RegionHandle;
            //m_velocity = cAgentData.Velocity;
        }

        public void CopyTo(AgentData cAgent)
        {
            cAgent.CallbackURI = m_callbackURI;

            cAgent.AgentID = UUID;
            cAgent.RegionID = Scene.RegionInfo.RegionID;

            cAgent.Position = AbsolutePosition;
            cAgent.Velocity = m_velocity;
            cAgent.Center = CameraPosition;
            cAgent.AtAxis = CameraAtAxis;
            cAgent.LeftAxis = CameraLeftAxis;
            cAgent.UpAxis = m_CameraUpAxis;

            cAgent.Far = DrawDistance;

            // Throttles 
            float multiplier = 1;
            int childRegions = KnownRegionCount;
            if (childRegions != 0)
                multiplier = 1f / childRegions;

            // Minimum throttle for a child region is 1/4 of the root region throttle
            if (multiplier <= 0.25f)
                multiplier = 0.25f;

            cAgent.Throttles = ControllingClient.GetThrottlesPacked(multiplier);

            cAgent.HeadRotation = m_headrotation;
            cAgent.BodyRotation = m_bodyRot;
            cAgent.ControlFlags = (uint)m_AgentControlFlags;

            if (m_scene.Permissions.IsGod(new UUID(cAgent.AgentID)))
                cAgent.GodLevel = (byte)GodLevel;
            else 
                cAgent.GodLevel = (byte) 0;

            cAgent.AlwaysRun = SetAlwaysRun;

            cAgent.Appearance = new AvatarAppearance(Appearance);
            
            lock (scriptedcontrols)
            {
                ControllerData[] controls = new ControllerData[scriptedcontrols.Count];
                int i = 0;

                foreach (ScriptControllers c in scriptedcontrols.Values)
                {
                    controls[i++] = new ControllerData(c.itemID, (uint)c.ignoreControls, (uint)c.eventControls);
                }
                cAgent.Controllers = controls;
            }

            // Animations
            try
            {
                cAgent.Anims = Animator.Animations.ToArray();
            }
            catch { }

            // Attachment objects
            lock (m_attachments)
            {
                if (m_attachments.Count > 0)
                {
                    cAgent.AttachmentObjects = new List<ISceneObject>();
                    cAgent.AttachmentObjectStates = new List<string>();
    //                IScriptModule se = m_scene.RequestModuleInterface<IScriptModule>();
                    InTransitScriptStates.Clear();

                    foreach (SceneObjectGroup sog in m_attachments)
                    {
                        // We need to make a copy and pass that copy
                        // because of transfers withn the same sim
                        ISceneObject clone = sog.CloneForNewScene();
                        // Attachment module assumes that GroupPosition holds the offsets...!
                        ((SceneObjectGroup)clone).RootPart.GroupPosition = sog.RootPart.AttachedPos;
                        ((SceneObjectGroup)clone).IsAttachment = false;
                        cAgent.AttachmentObjects.Add(clone);
                        string state = sog.GetStateSnapshot();
                        cAgent.AttachmentObjectStates.Add(state);
                        InTransitScriptStates.Add(state);
                        // Let's remove the scripts of the original object here
                        sog.RemoveScriptInstances(true);
                    }
                }
            }
        }

        private void CopyFrom(AgentData cAgent)
        {
            m_originRegionID = cAgent.RegionID;

            m_callbackURI = cAgent.CallbackURI;

            m_pos = cAgent.Position;
            m_velocity = cAgent.Velocity;
            CameraPosition = cAgent.Center;
            CameraAtAxis = cAgent.AtAxis;
            CameraLeftAxis = cAgent.LeftAxis;
            m_CameraUpAxis = cAgent.UpAxis;

            // When we get to the point of re-computing neighbors everytime this
            // changes, then start using the agent's drawdistance rather than the 
            // region's draw distance.
            // DrawDistance = cAgent.Far;
            DrawDistance = Scene.DefaultDrawDistance;

            if ((cAgent.Throttles != null) && cAgent.Throttles.Length > 0)
                ControllingClient.SetChildAgentThrottle(cAgent.Throttles);

            m_headrotation = cAgent.HeadRotation;
            Rotation = cAgent.BodyRotation;
            m_AgentControlFlags = (AgentManager.ControlFlags)cAgent.ControlFlags; 

            if (m_scene.Permissions.IsGod(new UUID(cAgent.AgentID)))
                GodLevel = cAgent.GodLevel;
            SetAlwaysRun = cAgent.AlwaysRun;

            Appearance = new AvatarAppearance(cAgent.Appearance);
            if (PhysicsActor != null)
            {
                bool isFlying = PhysicsActor.Flying;
                Vector3 velocity = PhysicsActor.Velocity;
                RemoveFromPhysicalScene();
                AddToPhysicalScene(isFlying);
                if (PhysicsActor != null)
                    PhysicsActor.Velocity = velocity;
            }
            
            try
            {
                lock (scriptedcontrols)
                {
                    if (cAgent.Controllers != null)
                    {
                        scriptedcontrols.Clear();

                        foreach (ControllerData c in cAgent.Controllers)
                        {
                            ScriptControllers sc = new ScriptControllers();
                            sc.itemID = c.ItemID;
                            sc.ignoreControls = (ScriptControlled)c.IgnoreControls;
                            sc.eventControls = (ScriptControlled)c.EventControls;

                            scriptedcontrols[sc.itemID] = sc;
                        }
                    }
                }
            }
            catch { }

            // FIXME: Why is this null check necessary?  Where are the cases where we get a null Anims object?
            if (cAgent.Anims != null)
                Animator.Animations.FromArray(cAgent.Anims);

            if (cAgent.AttachmentObjects != null && cAgent.AttachmentObjects.Count > 0)
            {
                m_attachments = new List<SceneObjectGroup>();
                int i = 0;
                foreach (ISceneObject so in cAgent.AttachmentObjects)
                {
                    ((SceneObjectGroup)so).LocalId = 0;
                    ((SceneObjectGroup)so).RootPart.ClearUpdateSchedule();
                    so.SetState(cAgent.AttachmentObjectStates[i++], m_scene);
                    m_scene.IncomingCreateObject(so);
                }
            }
        }

        public bool CopyAgent(out IAgentData agent)
        {
            agent = new CompleteAgentData();
            CopyTo((AgentData)agent);
            return true;
        }

        #endregion Child Agent Updates

        /// <summary>
        /// Handles part of the PID controller function for moving an avatar.
        /// </summary>
        public void UpdateMovement()
        {
            if (m_forceToApply.HasValue)
            {
                Vector3 force = m_forceToApply.Value;

                Updated = true;

                Velocity = force;

                m_forceToApply = null;
            }
        }

        /// <summary>
        /// Adds a physical representation of the avatar to the Physics plugin
        /// </summary>
        public void AddToPhysicalScene(bool isFlying)
        {
//            m_log.DebugFormat(
//                "[SCENE PRESENCE]: Adding physics actor for {0}, ifFlying = {1} in {2}",
//                Name, isFlying, Scene.RegionInfo.RegionName);

            if (Appearance.AvatarHeight == 0)
                Appearance.SetHeight();

            PhysicsScene scene = m_scene.PhysicsScene;

            Vector3 pVec = AbsolutePosition;

            // Old bug where the height was in centimeters instead of meters
            PhysicsActor = scene.AddAvatar(LocalId, Firstname + "." + Lastname, pVec,
                                                 new Vector3(0f, 0f, Appearance.AvatarHeight), isFlying);

            scene.AddPhysicsActorTaint(PhysicsActor);
            //PhysicsActor.OnRequestTerseUpdate += SendTerseUpdateToAllClients;
            PhysicsActor.OnCollisionUpdate += PhysicsCollisionUpdate;
            PhysicsActor.OnOutOfBounds += OutOfBoundsCall; // Called for PhysicsActors when there's something wrong
            PhysicsActor.SubscribeEvents(500);
            PhysicsActor.LocalID = LocalId;

            SetHeight(Appearance.AvatarHeight);
        }

        private void OutOfBoundsCall(Vector3 pos)
        {
            //bool flying = PhysicsActor.Flying;
            //RemoveFromPhysicalScene();

            //AddToPhysicalScene(flying);
            if (ControllingClient != null)
                ControllingClient.SendAgentAlertMessage("Physics is having a problem with your avatar.  You may not be able to move until you relog.", true);
        }

        /// <summary>
        /// Event called by the physics plugin to tell the avatar about a collision.
        /// </summary>
        /// <remarks>
        /// This function is called continuously, even when there are no collisions.  If the avatar is walking on the
        /// ground or a prim then there will be collision information between the avatar and the surface.
        ///
        /// FIXME: However, we can't safely avoid calling this yet where there are no collisions without analyzing whether
        /// any part of this method is relying on an every-frame call.
        /// </remarks>
        /// <param name="e"></param>
        public void PhysicsCollisionUpdate(EventArgs e)
        {
            if (IsChildAgent)
                return;
            
            //if ((Math.Abs(Velocity.X) > 0.1e-9f) || (Math.Abs(Velocity.Y) > 0.1e-9f))
            // The Physics Scene will send updates every 500 ms grep: PhysicsActor.SubscribeEvents(
            // as of this comment the interval is set in AddToPhysicalScene
            if (Animator != null)
            {
//                if (m_updateCount > 0)
//                {
                    Animator.UpdateMovementAnimations();
//                    m_updateCount--;
//                }
            }

            CollisionEventUpdate collisionData = (CollisionEventUpdate)e;
            Dictionary<uint, ContactPoint> coldata = collisionData.m_objCollisionList;

            CollisionPlane = Vector4.UnitW;

//            // No collisions at all means we may be flying. Update always
//            // to make falling work
//            if (m_lastColCount != coldata.Count || coldata.Count == 0)
//            {
//                m_updateCount = UPDATE_COUNT;
//                m_lastColCount = coldata.Count;
//            }

            if (coldata.Count != 0 && Animator != null)
            {
                switch (Animator.CurrentMovementAnimation)
                {
                    case "STAND":
                    case "WALK":
                    case "RUN":
                    case "CROUCH":
                    case "CROUCHWALK":
                        {
                            ContactPoint lowest;
                            lowest.SurfaceNormal = Vector3.Zero;
                            lowest.Position = Vector3.Zero;
                            lowest.Position.Z = Single.NaN;

                            foreach (ContactPoint contact in coldata.Values)
                            {
                                if (Single.IsNaN(lowest.Position.Z) || contact.Position.Z < lowest.Position.Z)
                                {
                                    lowest = contact;
                                }
                            }

                            CollisionPlane = new Vector4(-lowest.SurfaceNormal, -Vector3.Dot(lowest.Position, lowest.SurfaceNormal));
                        }
                        break;
                }
            }

            if (Invulnerable)
                return;
            
            float starthealth = Health;
            uint killerObj = 0;
            foreach (uint localid in coldata.Keys)
            {
                SceneObjectPart part = Scene.GetSceneObjectPart(localid);

                if (part != null && part.ParentGroup.Damage != -1.0f)
                    Health -= part.ParentGroup.Damage;
                else
                {
                    if (coldata[localid].PenetrationDepth >= 0.10f)
                        Health -= coldata[localid].PenetrationDepth * 5.0f;
                }

                if (Health <= 0.0f)
                {
                    if (localid != 0)
                        killerObj = localid;
                }
                //m_log.Debug("[AVATAR]: Collision with localid: " + localid.ToString() + " at depth: " + coldata[localid].ToString());
            }
            //Health = 100;
            if (!Invulnerable)
            {
                if (starthealth != Health)
                {
                    ControllingClient.SendHealth(Health);
                }
                if (Health <= 0)
                    m_scene.EventManager.TriggerAvatarKill(killerObj, this);
            }
        }

        public void setHealthWithUpdate(float health)
        {
            Health = health;
            ControllingClient.SendHealth(Health);
        }

        public void Close()
        {
            if (!IsChildAgent)
                m_scene.AttachmentsModule.DeleteAttachmentsFromScene(this, false);

            // Clear known regions
            KnownRegions = new Dictionary<ulong, string>();

            lock (m_reprioritization_timer)
            {
                m_reprioritization_timer.Enabled = false;
                m_reprioritization_timer.Elapsed -= new ElapsedEventHandler(Reprioritize);
            }
            
            // I don't get it but mono crashes when you try to dispose of this timer,
            // unsetting the elapsed callback should be enough to allow for cleanup however.
            // m_reprioritizationTimer.Dispose(); 

            RemoveFromPhysicalScene();
            Animator.Close();
            Animator = null;
        }

        public void AddAttachment(SceneObjectGroup gobj)
        {
            lock (m_attachments)
            {
                // This may be true when the attachment comes back
                // from serialization after login. Clear it.
                gobj.IsDeleted = false;

                m_attachments.Add(gobj);
            }
        }

        /// <summary>
        /// Get all the presence's attachments.
        /// </summary>
        /// <returns>A copy of the list which contains the attachments.</returns>
        public List<SceneObjectGroup> GetAttachments()
        {
            lock (m_attachments)
                return new List<SceneObjectGroup>(m_attachments);
        }

        /// <summary>
        /// Get the scene objects attached to the given point.
        /// </summary>
        /// <param name="attachmentPoint"></param>
        /// <returns>Returns an empty list if there were no attachments at the point.</returns>
        public List<SceneObjectGroup> GetAttachments(uint attachmentPoint)
        {
            List<SceneObjectGroup> attachments = new List<SceneObjectGroup>();
            
            lock (m_attachments)
            {
                foreach (SceneObjectGroup so in m_attachments)
                {
                    if (attachmentPoint == so.AttachmentPoint)
                        attachments.Add(so);
                }
            }
            
            return attachments;
        }

        public bool HasAttachments()
        {
            lock (m_attachments)
                return m_attachments.Count > 0;
        }

        public bool HasScriptedAttachments()
        {
            lock (m_attachments)
            {
                foreach (SceneObjectGroup gobj in m_attachments)
                {
                    if (gobj != null)
                    {
                        if (gobj.RootPart.Inventory.ContainsScripts())
                            return true;
                    }
                }
            }
            return false;
        }

        public void RemoveAttachment(SceneObjectGroup gobj)
        {
            lock (m_attachments)
                m_attachments.Remove(gobj);
        }

        /// <summary>
        /// Clear all attachments
        /// </summary>
        public void ClearAttachments()
        {
            lock (m_attachments)
                m_attachments.Clear();
        }

        /// <summary>
        /// This is currently just being done for information.
        /// </summary>
        public bool ValidateAttachments()
        {
            bool validated = true;

            lock (m_attachments)
            {
                // Validate
                foreach (SceneObjectGroup gobj in m_attachments)
                {
                    if (gobj == null)
                    {
                        m_log.WarnFormat(
                            "[SCENE PRESENCE]: Failed to validate an attachment for {0} since it was null.  Continuing", Name);

                        validated = false;
                    }
                    else if (gobj.IsDeleted)
                    {
                        m_log.WarnFormat(
                            "[SCENE PRESENCE]: Failed to validate attachment {0} {1} for {2} since it had been deleted.  Continuing",
                            gobj.Name, gobj.UUID, Name);

                        validated = false;
                    }
                }
            }

            return validated;
        }

        /// <summary>
        /// Send a script event to this scene presence's attachments
        /// </summary>
        /// <param name="eventName">The name of the event</param>
        /// <param name="args">The arguments for the event</param>
        public void SendScriptEventToAttachments(string eventName, Object[] args)
        {
            if (m_scriptEngines.Length == 0)
                return;

            lock (m_attachments)
            {
                foreach (SceneObjectGroup grp in m_attachments)
                {
                    // 16384 is CHANGED_ANIMATION
                    //
                    // Send this to all attachment root prims
                    //
                    foreach (IScriptModule m in m_scriptEngines)
                    {
                        if (m == null) // No script engine loaded
                            continue;

                        m.PostObjectEvent(grp.RootPart.UUID, "changed", new Object[] { (int)Changed.ANIMATION });
                    }
                }
            }
        }

        internal void PushForce(Vector3 impulse)
        {
            if (PhysicsActor != null)
            {
                PhysicsActor.AddForce(impulse,true);
            }
        }

        public void RegisterControlEventsToScript(int controls, int accept, int pass_on, uint Obj_localID, UUID Script_item_UUID)
        {
            ScriptControllers obj = new ScriptControllers();
            obj.ignoreControls = ScriptControlled.CONTROL_ZERO;
            obj.eventControls = ScriptControlled.CONTROL_ZERO;

            obj.itemID = Script_item_UUID;
            if (pass_on == 0 && accept == 0)
            {
                IgnoredControls |= (ScriptControlled)controls;
                obj.ignoreControls = (ScriptControlled)controls;
            }

            if (pass_on == 0 && accept == 1)
            {
                IgnoredControls |= (ScriptControlled)controls;
                obj.ignoreControls = (ScriptControlled)controls;
                obj.eventControls = (ScriptControlled)controls;
            }

            if (pass_on == 1 && accept == 1)
            {
                IgnoredControls = ScriptControlled.CONTROL_ZERO;
                obj.eventControls = (ScriptControlled)controls;
                obj.ignoreControls = ScriptControlled.CONTROL_ZERO;
            }

            lock (scriptedcontrols)
            {
                if (pass_on == 1 && accept == 0)
                {
                    IgnoredControls &= ~(ScriptControlled)controls;
                    if (scriptedcontrols.ContainsKey(Script_item_UUID))
                        scriptedcontrols.Remove(Script_item_UUID);
                }
                else
                {
                    scriptedcontrols[Script_item_UUID] = obj;
                }
            }

            ControllingClient.SendTakeControls(controls, pass_on == 1 ? true : false, true);
        }

        public void HandleForceReleaseControls(IClientAPI remoteClient, UUID agentID)
        {
            IgnoredControls = ScriptControlled.CONTROL_ZERO;
            lock (scriptedcontrols)
            {
                scriptedcontrols.Clear();
            }
            ControllingClient.SendTakeControls(int.MaxValue, false, false);
        }

        public void UnRegisterControlEventsToScript(uint Obj_localID, UUID Script_item_UUID)
        {
            ScriptControllers takecontrols;

            lock (scriptedcontrols)
            {
                if (scriptedcontrols.TryGetValue(Script_item_UUID, out takecontrols))
                {
                    ScriptControlled sctc = takecontrols.eventControls;

                    ControllingClient.SendTakeControls((int)sctc, false, false);
                    ControllingClient.SendTakeControls((int)sctc, true, false);

                    scriptedcontrols.Remove(Script_item_UUID);
                    IgnoredControls = ScriptControlled.CONTROL_ZERO;
                    foreach (ScriptControllers scData in scriptedcontrols.Values)
                    {
                        IgnoredControls |= scData.ignoreControls;
                    }
                }
            }
        }

        internal void SendControlToScripts(uint flags)
        {
            ScriptControlled allflags = ScriptControlled.CONTROL_ZERO;

            if (MouseDown)
            {
                allflags = LastCommands & (ScriptControlled.CONTROL_ML_LBUTTON | ScriptControlled.CONTROL_LBUTTON);
                if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_LBUTTON_UP) != 0 || (flags & unchecked((uint)AgentManager.ControlFlags.AGENT_CONTROL_ML_LBUTTON_UP)) != 0)
                {
                    allflags = ScriptControlled.CONTROL_ZERO;
                    MouseDown = true;
                }
            }

            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_ML_LBUTTON_DOWN) != 0)
            {
                allflags |= ScriptControlled.CONTROL_ML_LBUTTON;
                MouseDown = true;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_LBUTTON_DOWN) != 0)
            {
                allflags |= ScriptControlled.CONTROL_LBUTTON;
                MouseDown = true;
            }

            // find all activated controls, whether the scripts are interested in them or not
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_AT_POS) != 0 || (flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_AT_POS) != 0)
            {
                allflags |= ScriptControlled.CONTROL_FWD;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_AT_NEG) != 0 || (flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_AT_NEG) != 0)
            {
                allflags |= ScriptControlled.CONTROL_BACK;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_UP_POS) != 0 || (flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_POS) != 0)
            {
                allflags |= ScriptControlled.CONTROL_UP;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_UP_NEG) != 0 || (flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_NEG) != 0)
            {
                allflags |= ScriptControlled.CONTROL_DOWN;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_LEFT_POS) != 0 || (flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_LEFT_POS) != 0)
            {
                allflags |= ScriptControlled.CONTROL_LEFT;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_LEFT_NEG) != 0 || (flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_LEFT_NEG) != 0)
            {
                allflags |= ScriptControlled.CONTROL_RIGHT;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_YAW_NEG) != 0)
            {
                allflags |= ScriptControlled.CONTROL_ROT_RIGHT;
            }
            if ((flags & (uint)AgentManager.ControlFlags.AGENT_CONTROL_YAW_POS) != 0)
            {
                allflags |= ScriptControlled.CONTROL_ROT_LEFT;
            }
            // optimization; we have to check per script, but if nothing is pressed and nothing changed, we can skip that
            if (allflags != ScriptControlled.CONTROL_ZERO || allflags != LastCommands)
            {
                lock (scriptedcontrols)
                {
                    foreach (KeyValuePair<UUID, ScriptControllers> kvp in scriptedcontrols)
                    {
                        UUID scriptUUID = kvp.Key;
                        ScriptControllers scriptControlData = kvp.Value;

                        ScriptControlled localHeld = allflags & scriptControlData.eventControls;     // the flags interesting for us
                        ScriptControlled localLast = LastCommands & scriptControlData.eventControls; // the activated controls in the last cycle
                        ScriptControlled localChange = localHeld ^ localLast;                        // the changed bits
                        if (localHeld != ScriptControlled.CONTROL_ZERO || localChange != ScriptControlled.CONTROL_ZERO)
                        {
                            // only send if still pressed or just changed
                            m_scene.EventManager.TriggerControlEvent(scriptUUID, UUID, (uint)localHeld, (uint)localChange);
                        }
                    }
                }
            }

            LastCommands = allflags;
        }

        internal static AgentManager.ControlFlags RemoveIgnoredControls(AgentManager.ControlFlags flags, ScriptControlled ignored)
        {
            if (ignored == ScriptControlled.CONTROL_ZERO)
                return flags;

            if ((ignored & ScriptControlled.CONTROL_BACK) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_AT_NEG | AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_AT_NEG);
            if ((ignored & ScriptControlled.CONTROL_FWD) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_AT_POS | AgentManager.ControlFlags.AGENT_CONTROL_AT_POS);
            if ((ignored & ScriptControlled.CONTROL_DOWN) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_UP_NEG | AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_NEG);
            if ((ignored & ScriptControlled.CONTROL_UP) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_POS | AgentManager.ControlFlags.AGENT_CONTROL_UP_POS);
            if ((ignored & ScriptControlled.CONTROL_LEFT) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_LEFT_POS | AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_LEFT_POS);
            if ((ignored & ScriptControlled.CONTROL_RIGHT) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_LEFT_NEG | AgentManager.ControlFlags.AGENT_CONTROL_LEFT_NEG);
            if ((ignored & ScriptControlled.CONTROL_ROT_LEFT) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_YAW_NEG);
            if ((ignored & ScriptControlled.CONTROL_ROT_RIGHT) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_YAW_POS);
            if ((ignored & ScriptControlled.CONTROL_ML_LBUTTON) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_ML_LBUTTON_DOWN);
            if ((ignored & ScriptControlled.CONTROL_LBUTTON) != 0)
                flags &= ~(AgentManager.ControlFlags.AGENT_CONTROL_LBUTTON_UP | AgentManager.ControlFlags.AGENT_CONTROL_LBUTTON_DOWN);

            //DIR_CONTROL_FLAG_FORWARD = AgentManager.ControlFlags.AGENT_CONTROL_AT_POS,
            //DIR_CONTROL_FLAG_BACK = AgentManager.ControlFlags.AGENT_CONTROL_AT_NEG,
            //DIR_CONTROL_FLAG_LEFT = AgentManager.ControlFlags.AGENT_CONTROL_LEFT_POS,
            //DIR_CONTROL_FLAG_RIGHT = AgentManager.ControlFlags.AGENT_CONTROL_LEFT_NEG,
            //DIR_CONTROL_FLAG_UP = AgentManager.ControlFlags.AGENT_CONTROL_UP_POS,
            //DIR_CONTROL_FLAG_DOWN = AgentManager.ControlFlags.AGENT_CONTROL_UP_NEG,
            //DIR_CONTROL_FLAG_DOWN_NUDGE = AgentManager.ControlFlags.AGENT_CONTROL_NUDGE_UP_NEG

            return flags;
        }

        private void ReprioritizeUpdates()
        {
            if (Scene.IsReprioritizationEnabled && Scene.UpdatePrioritizationScheme != UpdatePrioritizationSchemes.Time)
            {
                lock (m_reprioritization_timer)
                {
                    if (!m_reprioritizing)
                        m_reprioritization_timer.Enabled = m_reprioritizing = true;
                    else
                        m_reprioritization_called = true;
                }
            }
        }

        private void Reprioritize(object sender, ElapsedEventArgs e)
        {
            ControllingClient.ReprioritizeUpdates();

            lock (m_reprioritization_timer)
            {
                m_reprioritization_timer.Enabled = m_reprioritizing = m_reprioritization_called;
                m_reprioritization_called = false;
            }
        }
    }
}
