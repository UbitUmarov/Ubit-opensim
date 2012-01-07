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

//#define USE_DRAWSTUFF
//#define SPAM

using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using System.IO;
using System.Diagnostics;
using log4net;
using Nini.Config;
using OdeAPI;
#if USE_DRAWSTUFF
using ODEDrawstuff;
#endif
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;
using OpenMetaverse;

namespace OpenSim.Region.Physics.OdePlugin
{
    public enum StatusIndicators : int
    {
        Generic = 0,
        Start = 1,
        End = 2
    }

    public struct sCollisionData
    {
        public uint ColliderLocalId;
        public uint CollidedWithLocalId;
        public int NumberOfCollisions;
        public int CollisionType;
        public int StatusIndicator;
        public int lastframe;
    }

    [Flags]
    public enum CollisionCategories : int
    {
        Disabled = 0,
        Geom = 0x00000001,
        Body = 0x00000002,
        Space = 0x00000004,
        Character = 0x00000008,
        Land = 0x00000010,
        Water = 0x00000020,
        Wind = 0x00000040,
        Sensor = 0x00000080,
        Selected = 0x00000100
    }

    /// <summary>
    /// Material type for a primitive
    /// </summary>
    public enum Material : int
    {
        /// <summary></summary>
        Stone = 0,
        /// <summary></summary>
        Metal = 1,
        /// <summary></summary>
        Glass = 2,
        /// <summary></summary>
        Wood = 3,
        /// <summary></summary>
        Flesh = 4,
        /// <summary></summary>
        Plastic = 5,
        /// <summary></summary>
        Rubber = 6,

        light = 7 // compatibility with old viewers
    }

    public enum changes : int
    {
        Add = 0,                // arg null. finishs the prim creation. should be used internally only ( to remove later ?)
        Remove,
        Link,               // arg AuroraODEPrim new parent prim or null to delink. Makes the prim part of a object with prim parent as root
        //  or removes from a object if arg is null
        DeLink,
        Position,           // arg Vector3 new position in world coords. Changes prim position. Prim must know if it is root or child
        Orientation,        // arg Quaternion new orientation in world coords. Changes prim position. Prim must know it it is root or child
        PosOffset,          // not in use
        // arg Vector3 new position in local coords. Changes prim position in object
        OriOffset,          // not in use
        // arg Vector3 new position in local coords. Changes prim position in object
        Velocity,
        AngVelocity,
        Acceleration,
        Force,
        Torque,

        AddForce,
        AddAngForce,
        AngLock,

        Size,
        Shape,

        CollidesWater,
        VolumeDtc,

        Physical,
        Selected,
        disabled,
        building,

        Null             //keep this last used do dim the methods array. does nothing but pulsing the prim
    }

    public struct ODEchangeitem
    {
        public OdePrim prim;
        public OdeCharacter character;
        public changes what;
        public Object arg;
    }
     
    public class OdeScene : PhysicsScene
    {
        private readonly ILog m_log;
        // private Dictionary<string, sCollisionData> m_storedCollisions = new Dictionary<string, sCollisionData>();

        private Random fluidRandomizer = new Random(Environment.TickCount);

        const d.ContactFlags comumContactFlags = d.ContactFlags.SoftERP | d.ContactFlags.SoftCFM |d.ContactFlags.Approx1 | d.ContactFlags.Bounce;
        const float comumContactERP = 0.6f;
        const float comumContactCFM = 0.0001f;
        
        float frictionScale = 5.0f;
        
        float frictionMovementMult = 0.3f;

        float TerrainBounce = 0.3f;
        float TerrainFriction = 0.3f;

        public float AvatarBounce = 0.3f;
        public float AvatarFriction = 0;// 0.9f * 0.5f;

        private const uint m_regionWidth = Constants.RegionSize;
        private const uint m_regionHeight = Constants.RegionSize;

        public float ODE_STEPSIZE = 0.020f; // make it visible
        private float metersInSpace = 25.6f;
        private float m_timeDilation = 1.0f;

        public float gravityx = 0f;
        public float gravityy = 0f;
        public float gravityz = -9.8f;


        private float waterlevel = 0f;
        private int framecount = 0;

        internal IntPtr WaterGeom;

        public float avPIDD = 3200f; // make it visible
        public float avPIDP = 1400f; // make it visible
        private float avCapRadius = 0.37f;
        private float avDensity = 3f;
        private float avMovementDivisorWalk = 1.3f;
        private float avMovementDivisorRun = 0.8f;
        private float minimumGroundFlightOffset = 3f;
        public float maximumMassObject = 10000.01f;

        public bool meshSculptedPrim = true;
        public bool forceSimplePrimMeshing = false;

        public float meshSculptLOD = 32;
        public float MeshSculptphysicalLOD = 16;

        public float geomDefaultDensity = 10.000006836f;

        public int geomContactPointsStartthrottle = 3;
        public int geomUpdatesPerThrottledUpdate = 15;

        public float bodyPIDD = 35f;
        public float bodyPIDG = 25;

        public int geomCrossingFailuresBeforeOutofbounds = 6;

        public float bodyMotorJointMaxforceTensor = 2;

        public int bodyFramesAutoDisable = 20;

        private float[] _watermap;
        private bool m_filterCollisions = true;

        private d.NearCallback nearCallback;

        public d.TriCallback triCallback;
        public d.TriArrayCallback triArrayCallback;
        private readonly HashSet<OdeCharacter> _characters = new HashSet<OdeCharacter>();
        private readonly HashSet<OdePrim> _prims = new HashSet<OdePrim>();
        private readonly HashSet<OdePrim> _activeprims = new HashSet<OdePrim>();

        private readonly Object _taintedCharacterLock = new Object();
        private readonly HashSet<OdeCharacter> _taintedCharacterH = new HashSet<OdeCharacter>(); // faster verification of repeated character taints
        private readonly Queue<OdeCharacter> _taintedCharacterQ = new Queue<OdeCharacter>(); // character taints

        public OpenSim.Framework.LocklessQueue<ODEchangeitem> ChangesQueue = new OpenSim.Framework.LocklessQueue<ODEchangeitem>();

        /// <summary>
        /// A list of actors that should receive collision events.
        /// </summary>
        private readonly List<PhysicsActor> _collisionEventPrim = new List<PhysicsActor>();
        
        private readonly HashSet<OdeCharacter> _badCharacter = new HashSet<OdeCharacter>();
        public Dictionary<IntPtr, String> geom_name_map = new Dictionary<IntPtr, String>();
        public Dictionary<IntPtr, PhysicsActor> actor_name_map = new Dictionary<IntPtr, PhysicsActor>();
        private bool m_NINJA_physics_joints_enabled = false;
        //private Dictionary<String, IntPtr> jointpart_name_map = new Dictionary<String,IntPtr>();
        private readonly Dictionary<String, List<PhysicsJoint>> joints_connecting_actor = new Dictionary<String, List<PhysicsJoint>>();

        private float contactsurfacelayer = 0.001f;

        private int contactsPerCollision = 80;
        internal IntPtr ContactgeomsArray = IntPtr.Zero;
        private IntPtr GlobalContactsArray = IntPtr.Zero;

        const int maxContactsbeforedeath = 4000;
        private volatile int m_global_contactcount = 0;


        private readonly IntPtr contactgroup;

        public ContactData[] m_materialContactsData = new ContactData[8];

        private readonly List<PhysicsJoint> requestedJointsToBeCreated = new List<PhysicsJoint>(); // lock only briefly. accessed by external code (to request new joints) and by OdeScene.Simulate() to move those joints into pending/active
        private readonly List<PhysicsJoint> pendingJoints = new List<PhysicsJoint>(); // can lock for longer. accessed only by OdeScene.
        private readonly List<PhysicsJoint> activeJoints = new List<PhysicsJoint>(); // can lock for longer. accessed only by OdeScene.
        private readonly List<string> requestedJointsToBeDeleted = new List<string>(); // lock only briefly. accessed by external code (to request deletion of joints) and by OdeScene.Simulate() to move those joints out of pending/active
        private Object externalJointRequestsLock = new Object();
        private readonly Dictionary<String, PhysicsJoint> SOPName_to_activeJoint = new Dictionary<String, PhysicsJoint>();
        private readonly Dictionary<String, PhysicsJoint> SOPName_to_pendingJoint = new Dictionary<String, PhysicsJoint>();
        private readonly DoubleDictionary<Vector3, IntPtr, IntPtr> RegionTerrain = new DoubleDictionary<Vector3, IntPtr, IntPtr>();
        private readonly Dictionary<IntPtr, float[]> TerrainHeightFieldHeights = new Dictionary<IntPtr, float[]>();
        private readonly Dictionary<IntPtr, GCHandle> TerrainHeightFieldHeightsHandlers = new Dictionary<IntPtr, GCHandle>();
       
        private int m_physicsiterations = 10;
        private const float m_SkipFramesAtms = 0.40f; // Drop frames gracefully at a 400 ms lag
        private readonly PhysicsActor PANull = new NullPhysicsActor();
        private float step_time = 0.0f;

        public IntPtr world;

        private uint obj2LocalID = 0;
        private OdeCharacter cc1;
        private OdePrim cp1;
        private OdeCharacter cc2;
        private OdePrim cp2;

        // split the spaces acording to contents type
        // ActiveSpace contains characters and active prims
        // StaticSpace contains land and other that is mostly static in enviroment
        // this can contain subspaces, like the grid in staticspace
        // as now space only contains this 2 top spaces

        public IntPtr TopSpace; // the global space
        public IntPtr ActiveSpace; // space for active prims
        public IntPtr StaticSpace; // space for the static things around

        // some speedup variables
        private int spaceGridMaxX;
        private int spaceGridMaxY;
        private float spacesPerMeter;

        // split static geometry collision into a grid as before
        private IntPtr[,] staticPrimspace;

        private Object OdeLock;
        private static Object SimulationLock;

        public IMesher mesher;

        private IConfigSource m_config;

        public bool physics_logging = false;
        public int physics_logging_interval = 0;
        public bool physics_logging_append_existing_logfile = false;

        public d.Vector3 xyz = new d.Vector3(128.1640f, 128.3079f, 25.7600f);
        public d.Vector3 hpr = new d.Vector3(125.5000f, -17.0000f, 0.0000f);

        private Vector3 m_worldOffset = Vector3.Zero;
        public Vector2 WorldExtents = new Vector2((int)Constants.RegionSize, (int)Constants.RegionSize);
        private PhysicsScene m_parentScene = null;

        private ODERayCastRequestManager m_rayCastManager;

        /// <summary>
        /// Initiailizes the scene
        /// Sets many properties that ODE requires to be stable
        /// These settings need to be tweaked 'exactly' right or weird stuff happens.
        /// </summary>
        public OdeScene(string sceneIdentifier)
            {
            m_log 
                = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType.ToString() + "." + sceneIdentifier);

            Name = sceneIdentifier;

            OdeLock = new Object();
            SimulationLock = new Object();

            nearCallback = near;

            triCallback = TriCallback;
            triArrayCallback = TriArrayCallback;
            m_rayCastManager = new ODERayCastRequestManager(this);
            lock (OdeLock)
                {
                // Create the world and the first space
                try
                    {
                    world = d.WorldCreate();
                    TopSpace = d.HashSpaceCreate(IntPtr.Zero);

                    // now the major subspaces
                    ActiveSpace = d.HashSpaceCreate(TopSpace);
                    StaticSpace = d.HashSpaceCreate(TopSpace);
                    }
                catch
                    {
                    // i must RtC#FM 
                    }

                d.HashSpaceSetLevels(TopSpace, -2, 8); // cell sizes from .25 to 256 ?? need check what this really does
                d.HashSpaceSetLevels(ActiveSpace, -2, 8);
                d.HashSpaceSetLevels(StaticSpace, -2, 8);

                // demote to second level
                d.SpaceSetSublevel(ActiveSpace, 1);
                d.SpaceSetSublevel(StaticSpace, 1);

                contactgroup = d.JointGroupCreate(0);
                //contactgroup

                d.WorldSetAutoDisableFlag(world, false);
                #if USE_DRAWSTUFF
                
                Thread viewthread = new Thread(new ParameterizedThreadStart(startvisualization));
                viewthread.Start();
                #endif
            }

            _watermap = new float[258 * 258];
        }

#if USE_DRAWSTUFF
        public void startvisualization(object o)
        {
            ds.Functions fn;
            fn.version = ds.VERSION;
            fn.start = new ds.CallbackFunction(start);
            fn.step = new ds.CallbackFunction(step);
            fn.command = new ds.CallbackFunction(command);
            fn.stop = null;
            fn.path_to_textures = "./textures";
            string[] args = new string[0];
            ds.SimulationLoop(args.Length, args, 352, 288, ref fn);
        }
#endif

        // Initialize the mesh plugin
        public override void Initialise(IMesher meshmerizer, IConfigSource config, RegionInfo region )
        {
            mesher = meshmerizer;
            m_config = config;

            if (region != null)
            {
                WorldExtents.X = region.RegionSizeX;
                WorldExtents.Y = region.RegionSizeY;
            }
            // Defaults

            avPIDD = 2200.0f;
            avPIDP = 900.0f;

            int contactsPerCollision = 80;

            if (m_config != null)
            {
                IConfig physicsconfig = m_config.Configs["ODEPhysicsSettings"];
                if (physicsconfig != null)
                {
                    gravityx = physicsconfig.GetFloat("world_gravityx", 0f);
                    gravityy = physicsconfig.GetFloat("world_gravityy", 0f);
                    gravityz = physicsconfig.GetFloat("world_gravityz", -9.8f);

                    metersInSpace = physicsconfig.GetFloat("meters_in_small_space", 29.9f);

                    contactsurfacelayer = physicsconfig.GetFloat("world_contact_surface_layer", 0.001f);

                    ODE_STEPSIZE = physicsconfig.GetFloat("world_stepsize", 0.020f);
                    m_physicsiterations = physicsconfig.GetInt("world_internal_steps_without_collisions", 10);

                    avDensity = physicsconfig.GetFloat("av_density", avDensity);
                    avMovementDivisorWalk = physicsconfig.GetFloat("av_movement_divisor_walk", 1.3f);
                    avMovementDivisorRun = physicsconfig.GetFloat("av_movement_divisor_run", 0.8f);
                    avCapRadius = physicsconfig.GetFloat("av_capsule_radius", 0.37f);

                    contactsPerCollision = physicsconfig.GetInt("contacts_per_collision", 80);

                    geomContactPointsStartthrottle = physicsconfig.GetInt("geom_contactpoints_start_throttling", 3);
                    geomUpdatesPerThrottledUpdate = physicsconfig.GetInt("geom_updates_before_throttled_update", 15);
                    geomCrossingFailuresBeforeOutofbounds = physicsconfig.GetInt("geom_crossing_failures_before_outofbounds", 5);

                    geomDefaultDensity = physicsconfig.GetFloat("geometry_default_density", 10.000006836f);
                    bodyFramesAutoDisable = physicsconfig.GetInt("body_frames_auto_disable", 20);

                    bodyPIDD = physicsconfig.GetFloat("body_pid_derivative", 35f);
                    bodyPIDG = physicsconfig.GetFloat("body_pid_gain", 25f);

                    forceSimplePrimMeshing = physicsconfig.GetBoolean("force_simple_prim_meshing", forceSimplePrimMeshing);
                    meshSculptedPrim = physicsconfig.GetBoolean("mesh_sculpted_prim", true);
                    meshSculptLOD = physicsconfig.GetFloat("mesh_lod", 32f);
                    MeshSculptphysicalLOD = physicsconfig.GetFloat("mesh_physical_lod", 16f);
                    m_filterCollisions = physicsconfig.GetBoolean("filter_collisions", false);

                    if (Environment.OSVersion.Platform == PlatformID.Unix)
                    {
                        avPIDD = physicsconfig.GetFloat("av_pid_derivative_linux", 2200.0f);
                        avPIDP = physicsconfig.GetFloat("av_pid_proportional_linux", 900.0f);
                        bodyMotorJointMaxforceTensor = physicsconfig.GetFloat("body_motor_joint_maxforce_tensor_linux", 5f);
                    }
                    else
                    {
                        avPIDD = physicsconfig.GetFloat("av_pid_derivative_win", 2200.0f);
                        avPIDP = physicsconfig.GetFloat("av_pid_proportional_win", 900.0f);
                        bodyMotorJointMaxforceTensor = physicsconfig.GetFloat("body_motor_joint_maxforce_tensor_win", 5f);
                    }

                    physics_logging = physicsconfig.GetBoolean("physics_logging", false);
                    physics_logging_interval = physicsconfig.GetInt("physics_logging_interval", 0);
                    physics_logging_append_existing_logfile = physicsconfig.GetBoolean("physics_logging_append_existing_logfile", false);

                    m_NINJA_physics_joints_enabled = physicsconfig.GetBoolean("use_NINJA_physics_joints", false);
                    minimumGroundFlightOffset = physicsconfig.GetFloat("minimum_ground_flight_offset", 3f);
                    maximumMassObject = physicsconfig.GetFloat("maximum_mass_object", 10000.01f);
                }
            }

            ContactgeomsArray = Marshal.AllocHGlobal(contactsPerCollision * d.ContactGeom.unmanagedSizeOf);
            GlobalContactsArray = GlobalContactsArray = Marshal.AllocHGlobal(maxContactsbeforedeath * d.Contact.unmanagedSizeOf);

            m_materialContactsData[(int)Material.Stone].mu = frictionScale * 0.8f;
            m_materialContactsData[(int)Material.Stone].bounce = 0.4f;

            m_materialContactsData[(int)Material.Metal].mu = frictionScale * 0.3f;
            m_materialContactsData[(int)Material.Metal].bounce = 0.4f;

            m_materialContactsData[(int)Material.Glass].mu = frictionScale * 0.2f;
            m_materialContactsData[(int)Material.Glass].bounce = 0.7f;

            m_materialContactsData[(int)Material.Wood].mu = frictionScale * 0.6f;
            m_materialContactsData[(int)Material.Wood].bounce = 0.5f;

            m_materialContactsData[(int)Material.Flesh].mu = frictionScale * 0.9f;
            m_materialContactsData[(int)Material.Flesh].bounce = 0.3f;

            m_materialContactsData[(int)Material.Plastic].mu = frictionScale * 0.4f;
            m_materialContactsData[(int)Material.Plastic].bounce = 0.7f;

            m_materialContactsData[(int)Material.Rubber].mu = frictionScale * 0.9f;
            m_materialContactsData[(int)Material.Rubber].bounce = 0.95f;

            m_materialContactsData[(int)Material.light].mu = 0.0f;
            m_materialContactsData[(int)Material.light].bounce = 0.0f;

            TerrainFriction *= frictionScale;
//            AvatarFriction *= frictionScale;

            // Set the gravity,, don't disable things automatically (we set it explicitly on some things)

            d.WorldSetGravity(world, gravityx, gravityy, gravityz);
            d.WorldSetContactSurfaceLayer(world, contactsurfacelayer);

            d.WorldSetLinearDamping(world, 0.001f);
            d.WorldSetAngularDamping(world, 0.001f);
            d.WorldSetAngularDampingThreshold(world, 0f);
            d.WorldSetLinearDampingThreshold(world, 0f);
            d.WorldSetMaxAngularSpeed(world, 256f);

            d.WorldSetCFM(world,1e-6f); // a bit harder than default
            d.WorldSetERP(world, 0.6f); // higher than original

            // Set how many steps we go without running collision testing
            // This is in addition to the step size.
            // Essentially Steps * m_physicsiterations
            d.WorldSetQuickStepNumIterations(world, m_physicsiterations);
            d.WorldSetContactMaxCorrectingVel(world, 100.0f);

            spacesPerMeter = 1 / metersInSpace;
            spaceGridMaxX = (int)(WorldExtents.X * spacesPerMeter);
            spaceGridMaxY = (int)(WorldExtents.Y * spacesPerMeter);

            staticPrimspace = new IntPtr[spaceGridMaxX, spaceGridMaxY];

            // create all spaces now
            int i, j;
            IntPtr newspace;
            for (i = 0; i < spaceGridMaxX; i++)
                for (j = 0; j < spaceGridMaxY; j++)
                {
                    newspace = d.HashSpaceCreate(StaticSpace);
                    d.GeomSetCategoryBits(newspace, (int)CollisionCategories.Space);
                    waitForSpaceUnlock(newspace);
                    d.SpaceSetSublevel(newspace, 2);
                    d.HashSpaceSetLevels(newspace, -2, 8);
                    staticPrimspace[i, j] = newspace;
                }
            // let this now be real maximum values
            spaceGridMaxX--;
            spaceGridMaxY--;
        }

        internal void waitForSpaceUnlock(IntPtr space)
        {
            //if (space != IntPtr.Zero)
                //while (d.SpaceLockQuery(space)) { } // Wait and do nothing
        }

        #region Collision Detection

        // sets a global contact for a joint for contactgeom , and base contact description)

        private IntPtr CreateContacJoint(ref d.ContactGeom contactGeom, float mu, float bounce)
        {
            if (GlobalContactsArray == IntPtr.Zero || m_global_contactcount >= maxContactsbeforedeath)
                return IntPtr.Zero;

            d.Contact newcontact = new d.Contact();
            newcontact.geom.depth = contactGeom.depth;
            newcontact.geom.g1 = contactGeom.g1;
            newcontact.geom.g2 = contactGeom.g2;
            newcontact.geom.pos = contactGeom.pos;
            newcontact.geom.normal = contactGeom.normal;
            newcontact.geom.side1 = contactGeom.side1;
            newcontact.geom.side2 = contactGeom.side2;

            // this needs bounce also
            newcontact.surface.mode = comumContactFlags;
            newcontact.surface.mu = mu;
            newcontact.surface.bounce = bounce;
            newcontact.surface.soft_cfm = comumContactCFM;
            newcontact.surface.soft_erp = comumContactERP;

            IntPtr contact = new IntPtr(GlobalContactsArray.ToInt64() + (Int64)(m_global_contactcount * d.Contact.unmanagedSizeOf));
            Marshal.StructureToPtr(newcontact, contact, false);
            return d.JointCreateContactPtr(world, contactgroup, contact);
        }


        /// <summary>
        /// This is our near callback.  A geometry is near a body
        /// </summary>
        /// <param name="space">The space that contains the geoms.  Remember, spaces are also geoms</param>
        /// <param name="g1">a geometry or space</param>
        /// <param name="g2">another geometry or space</param>
        /// 

        private bool GetCurContactGeom(int index, ref d.ContactGeom newcontactgeom)
        {
            if (ContactgeomsArray == IntPtr.Zero || index >= contactsPerCollision)
                return false;

            IntPtr contactptr = new IntPtr(ContactgeomsArray.ToInt64() + (Int64)(index * d.ContactGeom.unmanagedSizeOf));
            newcontactgeom = (d.ContactGeom)Marshal.PtrToStructure(contactptr, typeof(d.ContactGeom));
            return true;
        }



        private void near(IntPtr space, IntPtr g1, IntPtr g2)
        {
            //  no lock here!  It's invoked from within Simulate(), which is thread-locked

            if (m_global_contactcount >= maxContactsbeforedeath)
                return;

            // Test if we're colliding a geom with a space.
            // If so we have to drill down into the space recursively


            if (g1 == IntPtr.Zero || g2 == IntPtr.Zero)
                return;

            if (d.GeomIsSpace(g1) || d.GeomIsSpace(g2))
            {
                // We'll be calling near recursivly if one
                // of them is a space to find all of the
                // contact points in the space
                try
                {
                    d.SpaceCollide2(g1, g2, IntPtr.Zero, nearCallback);
                }
                catch (AccessViolationException)
                {
                    m_log.Warn("[PHYSICS]: Unable to collide test a space");
                    return;
                }
                //here one should check collisions of geoms inside a space
                // but on each space we only should have geoms that not colide amoung each other
                // so we don't dig inside spaces
                return;
            }

            // get geom bodies to check if we already a joint contact
            // guess this shouldn't happen now
            IntPtr b1 = d.GeomGetBody(g1);
            IntPtr b2 = d.GeomGetBody(g2);

            // d.GeomClassID id = d.GeomGetClass(g1);

            // Figure out how many contact points we have
            int count = 0;
            try
            {
                // Colliding Geom To Geom
                // This portion of the function 'was' blatantly ripped off from BoxStack.cs

                if (g1 == g2)
                    return; // Can't collide with yourself

                if (b1 != IntPtr.Zero && b2 != IntPtr.Zero && d.AreConnectedExcluding(b1, b2, d.JointType.Contact))
                    return;

                count = d.CollidePtr(g1, g2, (contactsPerCollision & 0xffff), ContactgeomsArray, d.ContactGeom.unmanagedSizeOf);
            }
            catch (SEHException)
            {
                m_log.Error("[PHYSICS]: The Operating system shut down ODE because of corrupt memory.  This could be a result of really irregular terrain.  If this repeats continuously, restart using Basic Physics and terrain fill your terrain.  Restarting the sim.");
//                ode.drelease(world);
                base.TriggerPhysicsBasedRestart();
            }
            catch (Exception e)
            {
                m_log.WarnFormat("[PHYSICS]: Unable to collide test an object: {0}", e.Message);
                return;
            }

            // no contacts so done
            if (count == 0)
                return;

            // now we have a contact describing colision of 2 things
            // but with my changes now we don't know what they are
            // so code gets more complex now

            // try get physical actors 
            PhysicsActor p1;
            PhysicsActor p2;

            if (!actor_name_map.TryGetValue(g1, out p1))
            {
                p1 = PANull;
            }

            if (!actor_name_map.TryGetValue(g2, out p2))
            {
                p2 = PANull;
            }

            // update actors collision score
            if (p1.CollisionScore >= float.MaxValue - count)
                p1.CollisionScore = 0;
            p1.CollisionScore += count;

            if (p2.CollisionScore >= float.MaxValue - count)
                p2.CollisionScore = 0;
            p2.CollisionScore += count;

            // get geoms names
            String name1 = null;
            String name2 = null;

            if (!geom_name_map.TryGetValue(g1, out name1))
            {
                name1 = "null";
            }
            if (!geom_name_map.TryGetValue(g2, out name2))
            {
                name2 = "null";
            }

            ContactPoint maxDepthContact = new ContactPoint();
            d.ContactGeom curContact = new d.ContactGeom();

            float mu;
            float bounce;
            ContactData contactdata1;
            ContactData contactdata2;

            for (int i = 0; i < count; i++)
            {
                if (!GetCurContactGeom(i, ref curContact))
                    break;

                if(curContact.g1 == IntPtr.Zero)
                    curContact.g1 = g1;
                if(curContact.g2 == IntPtr.Zero)
                    curContact.g2 = g2;

//for debug                d.Quaternion qtmp = d.BodyGetQuaternion(b1);

                if (curContact.depth > maxDepthContact.PenetrationDepth)
                {
                    maxDepthContact = new ContactPoint(
                        new Vector3(curContact.pos.X, curContact.pos.Y, curContact.pos.Z),
                        new Vector3(curContact.normal.X, curContact.normal.Y, curContact.normal.Z),
                        curContact.depth
                        );
                }

                IntPtr Joint;

                // inform actors about colision

                if (p1 is OdeCharacter && p2.PhysicsActorType == (int)ActorTypes.Prim)
                    {
                        // Testing if the collision is at the feet of the avatar
                        if ((p1.Position.Z - curContact.pos.Z) > (p1.Size.Z - avCapRadius) * 0.5f)
                            p1.IsColliding = true;
                    }
                else
                {
                    p1.IsColliding = true;
                }

                switch (p2.PhysicsActorType)
                {
                    case (int)ActorTypes.Agent:
                        p1.CollidingObj = true;
                        break;
                    case (int)ActorTypes.Prim:
                        if (p1.Velocity.LengthSquared() > 0.0f)
                            p1.CollidingObj = true;
                        break;
                    case (int)ActorTypes.Unknown:
                        p1.CollidingGround = true;
                        break;
                    default:
                        p1.CollidingGround = true;
                        break;
                }

                if (p2 is OdeCharacter && p1.PhysicsActorType == (int)ActorTypes.Prim)
                    {
                        // Testing if the collision is at the feet of the avatar
                        if ((p2.Position.Z - curContact.pos.Z) > (p2.Size.Z - avCapRadius) * 0.5f)
                            p2.IsColliding = true;
                    }
                else
                {
                    p2.IsColliding = true;
                }

                switch (p1.PhysicsActorType)
                {
                    case (int)ActorTypes.Agent:
                        p2.CollidingObj = true;
                        break;
                    case (int)ActorTypes.Prim:
                        if (p2.Velocity.LengthSquared() > 0.0f)
                            p2.CollidingObj = true;
                        break;
                    case (int)ActorTypes.Unknown:
                        p2.CollidingGround = true;
                        break;
                    default:
                        p2.CollidingGround = true;
                        break;
                }

                if (m_global_contactcount >= maxContactsbeforedeath)
                    break;

                // we don't want prim or avatar to explode
                // not in use section, so removed  see older commits if needed

                // skip actors with volumeDetect
                Boolean skipThisContact = false;

                if ((p1 is OdePrim) && (((OdePrim)p1).m_isVolumeDetect))
                    skipThisContact = true;   // No collision on volume detect prims

                if (!skipThisContact && (p2 is OdePrim) && (((OdePrim)p2).m_isVolumeDetect))
                    skipThisContact = true;   // No collision on volume detect prims

                if (!skipThisContact && curContact.depth < 0f)
                    skipThisContact = true;

                //                if (!skipThisContact && checkDupe(curContact, p2.PhysicsActorType))
                //                    skipThisContact = true;

                Joint = IntPtr.Zero;
                if (!skipThisContact)
                {
                   
                    // If we're colliding against terrain
                    if (name1 == "Terrain")
                    {
                        // avatar to ground
                        /*  not done by ode
                        if (p2.PhysicsActorType == (int)ActorTypes.Agent)
                            {
                            // If we're moving
                            if (Math.Abs(p2.Velocity.X) > 0.01f || Math.Abs(p2.Velocity.Y) > 0.01f)
                                {
                                float mu = AvatarMovementTerrainContactSurf.mu;
                                float bounce = AvatarMovementTerrainContactSurf.bounce;
                                float soft_cfm = AvatarMovementTerrainContactSurf.soft_cfm;
                                float soft_erp = AvatarMovementTerrainContactSurf.soft_erp;
                                doJoint = SetGlobalContact(ref curContact, mu, bounce, soft_cfm, soft_erp);
                                }
                            else
                                {
                                // Use the non moving terrain contact
                                float mu = TerrainContactSurf.mu;
                                float bounce = TerrainContactSurf.bounce;
                                float soft_cfm = TerrainContactSurf.soft_cfm;
                                float soft_erp = TerrainContactSurf.soft_erp;
                                doJoint = SetGlobalContact(ref curContact, mu, bounce, soft_cfm, soft_erp);
                                }
                            }
                        else
                            */
                        if (p2.PhysicsActorType == (int)ActorTypes.Prim)
                        {
                            // prim terrain contact

                            contactdata2 = p2.ContactData;

                            bounce = contactdata2.bounce * TerrainBounce;

                            mu = (float)Math.Sqrt(contactdata2.mu * TerrainFriction);

                            if (Math.Abs(p2.Velocity.X) > 0.1f || Math.Abs(p2.Velocity.Y) > 0.1f)
                                mu *= frictionMovementMult;

                            Joint = CreateContacJoint(ref curContact, mu, bounce);
                        }
                    }

                    else if (name2 == "Terrain")
                    {
                        // avatar to ground
                        /*  not done by ode
                        if (p1.PhysicsActorType == (int)ActorTypes.Agent)
                            {
                            // If we're moving
                            if (Math.Abs(p1.Velocity.X) > 0.01f || Math.Abs(p1.Velocity.Y) > 0.01f)
                                {
                                // Use the movement terrain contact
                                float mu = AvatarMovementTerrainContactSurf.mu;
                                float bounce = AvatarMovementTerrainContactSurf.bounce;
                                float soft_cfm = AvatarMovementTerrainContactSurf.soft_cfm;
                                float soft_erp = AvatarMovementTerrainContactSurf.soft_erp;
                                doJoint = SetGlobalContact(ref curContact, mu, bounce, soft_cfm, soft_erp);
                                }
                            else
                                {
                                // Use the non moving terrain contact
                                float mu = TerrainContactSurf.mu;
                                float bounce = TerrainContactSurf.bounce;
                                float soft_cfm = TerrainContactSurf.soft_cfm;
                                float soft_erp = TerrainContactSurf.soft_erp;
                                doJoint = SetGlobalContact(ref curContact, mu, bounce, soft_cfm, soft_erp);
                                }
                            }
                        else
                            */
                        if (p1.PhysicsActorType == (int)ActorTypes.Prim)
                        {
                            // prim terrain contact

                            contactdata1 = p1.ContactData;

                            bounce = contactdata1.bounce * TerrainBounce;

                            mu = (float)Math.Sqrt(contactdata1.mu * TerrainFriction);

                            if (Math.Abs(p1.Velocity.X) > 0.1f || Math.Abs(p1.Velocity.Y) > 0.1f)
                                mu *= frictionMovementMult;

                            Joint = CreateContacJoint(ref curContact, mu, bounce);
                        }
                    }

                    // collisions with water
                    else if (name1 == "Water" || name2 == "Water")
                    {
                        if (curContact.depth > 0.1f)
                        {
                            curContact.depth *= 52;
                            //contact.normal = new d.Vector3(0, 0, 1);
                            //contact.pos = new d.Vector3(0, 0, contact.pos.Z - 5f);
                        }
                        mu = 0;
                        bounce = 0;
                        Joint = CreateContacJoint(ref curContact, mu, bounce);
                    }

                    else
                    {
                        // we're colliding with prim or avatar
                        if (p1 != null && p2 != null)
                        {
                            contactdata1 = p1.ContactData;
                            contactdata2 = p2.ContactData;

                            bounce = contactdata1.bounce * contactdata2.bounce;
                            
                            mu = (float)Math.Sqrt(contactdata1.mu * contactdata2.mu);

                            if ((Math.Abs(p2.Velocity.X - p1.Velocity.X) > 0.1f || Math.Abs(p2.Velocity.Y - p1.Velocity.Y) > 0.1f))
                                mu *= frictionMovementMult;
                            
                            Joint = CreateContacJoint(ref curContact, mu, bounce);
                        }
                    }
                    if (Joint != IntPtr.Zero)
                    {
                        m_global_contactcount++;
                        d.JointAttach(Joint, b1, b2);
                    }
                }
            }
            // this was inside above loop ?

            collision_accounting_events(p1, p2, maxDepthContact);

/*
            if (notskipedcount > geomContactPointsStartthrottle)
            {
                // If there are more then 3 contact points, it's likely
                // that we've got a pile of objects, so ...
                // We don't want to send out hundreds of terse updates over and over again
                // so lets throttle them and send them again after it's somewhat sorted out.
                 this needs checking so out for now
                                if (b1 != IntPtr.Zero)
                                    p1.ThrottleUpdates = true;
                                if (b2 != IntPtr.Zero)
                                    p2.ThrottleUpdates = true;
                
            }
 */
        }            

        private void collision_accounting_events(PhysicsActor p1, PhysicsActor p2, ContactPoint contact)
            {
            // obj1LocalID = 0;
            //returncollisions = false;
            obj2LocalID = 0;
            //ctype = 0;
            //cStartStop = 0;
            if (!(p2.SubscribedEvents() || p1.SubscribedEvents()))
                return;

            switch ((ActorTypes)p1.PhysicsActorType)
                {
                case ActorTypes.Agent:
                    cc1 = (OdeCharacter)p1;
                    switch ((ActorTypes)p2.PhysicsActorType)
                        {
                        case ActorTypes.Agent:
                            cc2 = (OdeCharacter)p2;
                            obj2LocalID = cc2.m_localID;
                            if (p2.SubscribedEvents())
                                cc2.AddCollisionEvent(cc1.m_localID, contact);
                            break;

                        case ActorTypes.Prim:
                            if (p2 is OdePrim)
                                {
                                cp2 = (OdePrim)p2;
                                obj2LocalID = cp2.m_localID;
                                if (p2.SubscribedEvents())
                                    cp2.AddCollisionEvent(cc1.m_localID, contact);
                                }
                            break;

                        case ActorTypes.Ground:
                        case ActorTypes.Unknown:
                        default:
                            obj2LocalID = 0;
                            break;
                        }
                    if (p1.SubscribedEvents())
                        {
                        contact.SurfaceNormal = -contact.SurfaceNormal;
                        cc1.AddCollisionEvent(obj2LocalID, contact);
                        }
                    break;

                case ActorTypes.Prim:

                    if (p1 is OdePrim)
                        {
                        cp1 = (OdePrim)p1;

                        // obj1LocalID = cp2.m_localID;
                        switch ((ActorTypes)p2.PhysicsActorType)
                            {
                            case ActorTypes.Agent:
                                if (p2 is OdeCharacter)
                                    {
                                    cc2 = (OdeCharacter)p2;
                                    obj2LocalID = cc2.m_localID;
                                    if (p2.SubscribedEvents())
                                        cc2.AddCollisionEvent(cp1.m_localID, contact);
                                    }
                                break;
                            case ActorTypes.Prim:

                                if (p2 is OdePrim)
                                    {
                                    cp2 = (OdePrim)p2;
                                    obj2LocalID = cp2.m_localID;
                                    if (p2.SubscribedEvents())
                                        cp2.AddCollisionEvent(cp1.m_localID, contact);
                                    }
                                break;

                            case ActorTypes.Ground:
                            case ActorTypes.Unknown:
                            default:
                                obj2LocalID = 0;
                                break;
                            }
                        if (p1.SubscribedEvents())
                            {
                            contact.SurfaceNormal = -contact.SurfaceNormal;
                            cp1.AddCollisionEvent(obj2LocalID, contact);
                            }
                        }
                    break;
                }
            }

        public int TriArrayCallback(IntPtr trimesh, IntPtr refObject, int[] triangleIndex, int triCount)
        {
            /*            String name1 = null;
                        String name2 = null;

                        if (!geom_name_map.TryGetValue(trimesh, out name1))
                        {
                            name1 = "null";
                        }
                        if (!geom_name_map.TryGetValue(refObject, out name2))
                        {
                            name2 = "null";
                        }

                        m_log.InfoFormat("TriArrayCallback: A collision was detected between {1} and {2}", 0, name1, name2);
            */
            return 1;
        }

        public int TriCallback(IntPtr trimesh, IntPtr refObject, int triangleIndex)
        {
//            String name1 = null;
//            String name2 = null;
//
//            if (!geom_name_map.TryGetValue(trimesh, out name1))
//            {
//                name1 = "null";
//            }
//
//            if (!geom_name_map.TryGetValue(refObject, out name2))
//            {
//                name2 = "null";
//            }

            //            m_log.InfoFormat("TriCallback: A collision was detected between {1} and {2}. Index was {3}", 0, name1, name2, triangleIndex);

            d.Vector3 v0 = new d.Vector3();
            d.Vector3 v1 = new d.Vector3();
            d.Vector3 v2 = new d.Vector3();

            d.GeomTriMeshGetTriangle(trimesh, 0, ref v0, ref v1, ref v2);
            //            m_log.DebugFormat("Triangle {0} is <{1},{2},{3}>, <{4},{5},{6}>, <{7},{8},{9}>", triangleIndex, v0.X, v0.Y, v0.Z, v1.X, v1.Y, v1.Z, v2.X, v2.Y, v2.Z);

            return 1;
        }

        /// <summary>
        /// This is our collision testing routine in ODE
        /// </summary>
        /// <param name="timeStep"></param>
        private void collision_optimized(float timeStep)
        {
//        _perloopContact.Clear();
// clear characts IsColliding until we do it some other way

            lock (_characters)
                {
                foreach (OdeCharacter chr in _characters)
                    {
                    // this are odd checks  if they are needed something is wrong elsewhere
                    // keep for now
                    if (chr == null)
                        continue;

                    if (chr.Shell == IntPtr.Zero || chr.Body == IntPtr.Zero)
                        continue;

                    chr.IsColliding = false;
                    //                    chr.CollidingGround = false; not done here
                    chr.CollidingObj = false;
                    }
                }

            // now let ode do its job
            // colide active things amoung them
            try
                {
                d.SpaceCollide(ActiveSpace, IntPtr.Zero, nearCallback);
                }
            catch (AccessViolationException)
                {
                m_log.Warn("[PHYSICS]: Unable to Active space collide");
                }

            // then active things with static enviroment
            try
                {
                d.SpaceCollide2(ActiveSpace,StaticSpace, IntPtr.Zero, nearCallback);
                }
            catch (AccessViolationException)
                {
                m_log.Warn("[PHYSICS]: Unable to Active to static space collide");
                }

//            _perloopContact.Clear();
        }

        #endregion


        public float GetTerrainHeightAtXY(float x, float y)
        {
            // assumes 1m size grid and constante size square regions
            // region offset in mega position

            int offsetX = ((int)(x / (int)Constants.RegionSize)) * (int)Constants.RegionSize;
            int offsetY = ((int)(y / (int)Constants.RegionSize)) * (int)Constants.RegionSize;

            IntPtr heightFieldGeom = IntPtr.Zero;

            // get region map
            if (!RegionTerrain.TryGetValue(new Vector3(offsetX, offsetY, 0), out heightFieldGeom))
                return 0f;

            if (heightFieldGeom == IntPtr.Zero)
                return 0f;

            if (!TerrainHeightFieldHeights.ContainsKey(heightFieldGeom))
                return 0f;

            // TerrainHeightField for ODE as offset 1m
            x += 1f - offsetX;
            y += 1f - offsetY;

            // make position fit into array
            if (x < 0)
                x = 0;
            if (y < 0)
                y = 0;

            // integer indexs
            int ix;
            int iy;
            //  interpolators offset
            float dx;
            float dy;

            int regsize = (int)Constants.RegionSize + 2; // map size see setterrain

            // we  still have square fixed size regions
            // also flip x and y because of how map is done for ODE fliped axis
            // so ix,iy,dx and dy are inter exchanged
            if (x < regsize - 1)
            {
                iy = (int)x;
                dy = x - (float)iy;
            }
            else // out world use external height
            {
                iy = regsize - 1;
                dy = 0;
            }
            if (y < regsize - 1)
            {
                ix = (int)y;
                dx = y - (float)ix;
            }
            else
            {
                ix = regsize - 1;
                dx = 0;
            }

            float h0;
            float h1;
            float h2;

            iy *= regsize;
            iy += ix; // all indexes have iy + ix

            float[] heights = TerrainHeightFieldHeights[heightFieldGeom];

            if ((dx + dy) <= 1.0f)
            {
                h0 = ((float)heights[iy]); // 0,0 vertice
                h1 = (((float)heights[iy + 1]) - h0) * dx; // 1,0 vertice minus 0,0
                h2 = (((float)heights[iy + regsize]) - h0) * dy; // 0,1 vertice minus 0,0
            }
            else
            {
                h0 = ((float)heights[iy + regsize + 1]); // 1,1 vertice
                h1 = (((float)heights[iy + 1]) - h0) * (1 - dy); // 1,1 vertice minus 1,0
                h2 = (((float)heights[iy + regsize]) - h0) * (1 - dx); // 1,1 vertice minus 0,1
            }

            return h0 + h1 + h2;
        }

        /// <summary>
        /// Add actor to the list that should receive collision events in the simulate loop.
        /// </summary>
        /// <param name="obj"></param>
        public void AddCollisionEventReporting(PhysicsActor obj)
        {
            lock (_collisionEventPrim)
            {
                if (!_collisionEventPrim.Contains(obj))
                    _collisionEventPrim.Add(obj);
            }
        }

        /// <summary>
        /// Remove actor from the list that should receive collision events in the simulate loop.
        /// </summary>
        /// <param name="obj"></param>
        public void RemoveCollisionEventReporting(PhysicsActor obj)
        {
            lock (_collisionEventPrim)
            {
                if (!_collisionEventPrim.Contains(obj))
                    _collisionEventPrim.Remove(obj);
            }
        }

        #region Add/Remove Entities

        public override PhysicsActor AddAvatar(string avName, Vector3 position, Vector3 size, bool isFlying)
        {
            Vector3 pos;
            pos.X = position.X;
            pos.Y = position.Y;
            pos.Z = position.Z;
            OdeCharacter newAv = new OdeCharacter(avName, this, pos, size, avPIDD, avPIDP, avCapRadius, avDensity, avMovementDivisorWalk, avMovementDivisorRun);
            newAv.Flying = isFlying;
            newAv.MinimumGroundFlightOffset = minimumGroundFlightOffset;
            
            return newAv;
        }

        public void AddCharacter(OdeCharacter chr)
        {
            lock (_characters)
            {
                if (!_characters.Contains(chr))
                {
                    _characters.Add(chr);
                    if (chr.bad)
                        m_log.DebugFormat("[PHYSICS] Added BAD actor {0} to characters list", chr.m_uuid);
                }
            }
        }

        public void RemoveCharacter(OdeCharacter chr)
        {
            lock (_characters)
            {
                if (_characters.Contains(chr))
                {
                    _characters.Remove(chr);
                }
            }
        }

        public void BadCharacter(OdeCharacter chr)
        {
            lock (_badCharacter)
            {
                if (!_badCharacter.Contains(chr))
                    _badCharacter.Add(chr);
            }
        }

        public override void RemoveAvatar(PhysicsActor actor)
        {
            //m_log.Debug("[PHYSICS]:ODELOCK");
            ((OdeCharacter) actor).Destroy();
        }

        private PhysicsActor AddPrim(String name, Vector3 position, Vector3 size, Quaternion rotation,
                                     PrimitiveBaseShape pbs, bool isphysical, uint localID)
        {
            Vector3 pos = position;
            Vector3 siz = size;
            Quaternion rot = rotation;

            OdePrim newPrim;
            lock (OdeLock)
            {
                newPrim = new OdePrim(name, this, pos, siz, rot, pbs, isphysical);

                lock (_prims)
                    _prims.Add(newPrim);
            }
            newPrim.LocalID = localID;
            return newPrim;
        }

        public void addActivePrim(OdePrim activatePrim)
        {
            // adds active prim..   (ones that should be iterated over in collisions_optimized
            lock (_activeprims)
            {
                if (!_activeprims.Contains(activatePrim))
                    _activeprims.Add(activatePrim);
                //else
                  //  m_log.Warn("[PHYSICS]: Double Entry in _activeprims detected, potential crash immenent");
            }
        }

        public override PhysicsActor AddPrimShape(string primName, PrimitiveBaseShape pbs, Vector3 position,
                                                  Vector3 size, Quaternion rotation, bool isPhysical, uint localid)
        {
#if SPAM
            m_log.DebugFormat("[PHYSICS]: Adding physics actor to {0}", primName);
#endif

            return AddPrim(primName, position, size, rotation, pbs, isPhysical, localid);
        }

        public override float TimeDilation
        {
            get { return m_timeDilation; }
        }

        public override bool SupportsNINJAJoints
        {
            get { return m_NINJA_physics_joints_enabled; }
        }

        // internal utility function: must be called within a lock (OdeLock)
        private void InternalAddActiveJoint(PhysicsJoint joint)
        {
            activeJoints.Add(joint);
            SOPName_to_activeJoint.Add(joint.ObjectNameInScene, joint);
        }

        // internal utility function: must be called within a lock (OdeLock)
        private void InternalAddPendingJoint(OdePhysicsJoint joint)
        {
            pendingJoints.Add(joint);
            SOPName_to_pendingJoint.Add(joint.ObjectNameInScene, joint);
        }

        // internal utility function: must be called within a lock (OdeLock)
        private void InternalRemovePendingJoint(PhysicsJoint joint)
        {
            pendingJoints.Remove(joint);
            SOPName_to_pendingJoint.Remove(joint.ObjectNameInScene);
        }

        // internal utility function: must be called within a lock (OdeLock)
        private void InternalRemoveActiveJoint(PhysicsJoint joint)
        {
            activeJoints.Remove(joint);
            SOPName_to_activeJoint.Remove(joint.ObjectNameInScene);
        }

        public override void DumpJointInfo()
        {
            string hdr = "[NINJA] JOINTINFO: ";
            foreach (PhysicsJoint j in pendingJoints)
            {
                m_log.Debug(hdr + " pending joint, Name: " + j.ObjectNameInScene + " raw parms:" + j.RawParams);
            }
            m_log.Debug(hdr + pendingJoints.Count + " total pending joints");
            foreach (string jointName in SOPName_to_pendingJoint.Keys)
            {
                m_log.Debug(hdr + " pending joints dict contains Name: " + jointName);
            }
            m_log.Debug(hdr + SOPName_to_pendingJoint.Keys.Count + " total pending joints dict entries");
            foreach (PhysicsJoint j in activeJoints)
            {
                m_log.Debug(hdr + " active joint, Name: " + j.ObjectNameInScene + " raw parms:" + j.RawParams);
            }
            m_log.Debug(hdr + activeJoints.Count + " total active joints");
            foreach (string jointName in SOPName_to_activeJoint.Keys)
            {
                m_log.Debug(hdr + " active joints dict contains Name: " + jointName);
            }
            m_log.Debug(hdr + SOPName_to_activeJoint.Keys.Count + " total active joints dict entries");

            m_log.Debug(hdr + " Per-body joint connectivity information follows.");
            m_log.Debug(hdr + joints_connecting_actor.Keys.Count + " bodies are connected by joints.");
            foreach (string actorName in joints_connecting_actor.Keys)
            {
                m_log.Debug(hdr + " Actor " + actorName + " has the following joints connecting it");
                foreach (PhysicsJoint j in joints_connecting_actor[actorName])
                {
                    m_log.Debug(hdr + " * joint Name: " + j.ObjectNameInScene + " raw parms:" + j.RawParams);
                }
                m_log.Debug(hdr + joints_connecting_actor[actorName].Count + " connecting joints total for this actor");
            }
        }

        public override void RequestJointDeletion(string ObjectNameInScene)
        {
            lock (externalJointRequestsLock)
            {
                if (!requestedJointsToBeDeleted.Contains(ObjectNameInScene)) // forbid same deletion request from entering twice to prevent spurious deletions processed asynchronously
                {
                    requestedJointsToBeDeleted.Add(ObjectNameInScene);
                }
            }
        }

        private void DeleteRequestedJoints()
        {
            List<string> myRequestedJointsToBeDeleted;
            lock (externalJointRequestsLock)
            {
                // make a local copy of the shared list for processing (threading issues)
                myRequestedJointsToBeDeleted = new List<string>(requestedJointsToBeDeleted);
            }

            foreach (string jointName in myRequestedJointsToBeDeleted)
            {
                lock (OdeLock)
                {
                    //m_log.Debug("[NINJA] trying to deleting requested joint " + jointName);
                    if (SOPName_to_activeJoint.ContainsKey(jointName) || SOPName_to_pendingJoint.ContainsKey(jointName))
                    {
                        OdePhysicsJoint joint = null;
                        if (SOPName_to_activeJoint.ContainsKey(jointName))
                        {
                            joint = SOPName_to_activeJoint[jointName] as OdePhysicsJoint;
                            InternalRemoveActiveJoint(joint);
                        }
                        else if (SOPName_to_pendingJoint.ContainsKey(jointName))
                        {
                            joint = SOPName_to_pendingJoint[jointName] as OdePhysicsJoint;
                            InternalRemovePendingJoint(joint);
                        }

                        if (joint != null)
                        {
                            //m_log.Debug("joint.BodyNames.Count is " + joint.BodyNames.Count + " and contents " + joint.BodyNames);
                            for (int iBodyName = 0; iBodyName < 2; iBodyName++)
                            {
                                string bodyName = joint.BodyNames[iBodyName];
                                if (bodyName != "NULL")
                                {
                                    joints_connecting_actor[bodyName].Remove(joint);
                                    if (joints_connecting_actor[bodyName].Count == 0)
                                    {
                                        joints_connecting_actor.Remove(bodyName);
                                    }
                                }
                            }

                            DoJointDeactivated(joint);
                            if (joint.jointID != IntPtr.Zero)
                            {
                                d.JointDestroy(joint.jointID);
                                joint.jointID = IntPtr.Zero;
                                //DoJointErrorMessage(joint, "successfully destroyed joint " + jointName);
                            }
                            else
                            {
                                //m_log.Warn("[NINJA] Ignoring re-request to destroy joint " + jointName);
                            }
                        }
                        else
                        {
                            // DoJointErrorMessage(joint, "coult not find joint to destroy based on name " + jointName);
                        }
                    }
                    else
                    {
                        // DoJointErrorMessage(joint, "WARNING - joint removal failed, joint " + jointName);
                    }
                }
            }

            // remove processed joints from the shared list
            lock (externalJointRequestsLock)
            {
                foreach (string jointName in myRequestedJointsToBeDeleted)
                {
                    requestedJointsToBeDeleted.Remove(jointName);
                }
            }
        }

        // for pending joints we don't know if their associated bodies exist yet or not.
        // the joint is actually created during processing of the taints
        private void CreateRequestedJoints()
        {
            List<PhysicsJoint> myRequestedJointsToBeCreated;
            lock (externalJointRequestsLock)
            {
                // make a local copy of the shared list for processing (threading issues)
                myRequestedJointsToBeCreated = new List<PhysicsJoint>(requestedJointsToBeCreated);
            }

            foreach (PhysicsJoint joint in myRequestedJointsToBeCreated)
            {
                lock (OdeLock)
                {
                    if (SOPName_to_pendingJoint.ContainsKey(joint.ObjectNameInScene) && SOPName_to_pendingJoint[joint.ObjectNameInScene] != null)
                    {
                        DoJointErrorMessage(joint, "WARNING: ignoring request to re-add already pending joint Name:" + joint.ObjectNameInScene + " type:" + joint.Type + " parms: " + joint.RawParams + " pos: " + joint.Position + " rot:" + joint.Rotation);
                        continue;
                    }
                    if (SOPName_to_activeJoint.ContainsKey(joint.ObjectNameInScene) && SOPName_to_activeJoint[joint.ObjectNameInScene] != null)
                    {
                        DoJointErrorMessage(joint, "WARNING: ignoring request to re-add already active joint Name:" + joint.ObjectNameInScene + " type:" + joint.Type + " parms: " + joint.RawParams + " pos: " + joint.Position + " rot:" + joint.Rotation);
                        continue;
                    }

                    InternalAddPendingJoint(joint as OdePhysicsJoint);

                    if (joint.BodyNames.Count >= 2)
                    {
                        for (int iBodyName = 0; iBodyName < 2; iBodyName++)
                        {
                            string bodyName = joint.BodyNames[iBodyName];
                            if (bodyName != "NULL")
                            {
                                if (!joints_connecting_actor.ContainsKey(bodyName))
                                {
                                    joints_connecting_actor.Add(bodyName, new List<PhysicsJoint>());
                                }
                                joints_connecting_actor[bodyName].Add(joint);
                            }
                        }
                    }
                }
            }

            // remove processed joints from shared list
            lock (externalJointRequestsLock)
            {
                foreach (PhysicsJoint joint in myRequestedJointsToBeCreated)
                {
                    requestedJointsToBeCreated.Remove(joint);
                }
            }
        }

        /// <summary>
        /// Add a request for joint creation.
        /// </summary>
        /// <remarks>
        /// this joint will just be added to a waiting list that is NOT processed during the main
        /// Simulate() loop (to avoid deadlocks). After Simulate() is finished, we handle unprocessed joint requests.
        /// </remarks>
        /// <param name="objectNameInScene"></param>
        /// <param name="jointType"></param>
        /// <param name="position"></param>
        /// <param name="rotation"></param>
        /// <param name="parms"></param>
        /// <param name="bodyNames"></param>
        /// <param name="trackedBodyName"></param>
        /// <param name="localRotation"></param>
        /// <returns></returns>
        public override PhysicsJoint RequestJointCreation(
            string objectNameInScene, PhysicsJointType jointType, Vector3 position,
            Quaternion rotation, string parms, List<string> bodyNames, string trackedBodyName, Quaternion localRotation)
        {
            OdePhysicsJoint joint = new OdePhysicsJoint();
            joint.ObjectNameInScene = objectNameInScene;
            joint.Type = jointType;
            joint.Position = position;
            joint.Rotation = rotation;
            joint.RawParams = parms;
            joint.BodyNames = new List<string>(bodyNames);
            joint.TrackedBodyName = trackedBodyName;
            joint.LocalRotation = localRotation;
            joint.jointID = IntPtr.Zero;
            joint.ErrorMessageCount = 0;

            lock (externalJointRequestsLock)
            {
                if (!requestedJointsToBeCreated.Contains(joint)) // forbid same creation request from entering twice 
                {
                    requestedJointsToBeCreated.Add(joint);
                }
            }

            return joint;
        }

        private void RemoveAllJointsConnectedToActor(PhysicsActor actor)
        {
            //m_log.Debug("RemoveAllJointsConnectedToActor: start");
            if (actor.SOPName != null && joints_connecting_actor.ContainsKey(actor.SOPName) && joints_connecting_actor[actor.SOPName] != null)
            {

                List<PhysicsJoint> jointsToRemove = new List<PhysicsJoint>();
                //TODO: merge these 2 loops (originally it was needed to avoid altering a list being iterated over, but it is no longer needed due to the joint request queue mechanism)
                foreach (PhysicsJoint j in joints_connecting_actor[actor.SOPName])
                {
                    jointsToRemove.Add(j);
                }
                foreach (PhysicsJoint j in jointsToRemove)
                {
                    //m_log.Debug("RemoveAllJointsConnectedToActor: about to request deletion of " + j.ObjectNameInScene);
                    RequestJointDeletion(j.ObjectNameInScene);
                    //m_log.Debug("RemoveAllJointsConnectedToActor: done request deletion of " + j.ObjectNameInScene);
                    j.TrackedBodyName = null; // *IMMEDIATELY* prevent any further movement of this joint (else a deleted actor might cause spurious tracking motion of the joint for a few frames, leading to the joint proxy object disappearing)
                }
            }
        }

        public override void RemoveAllJointsConnectedToActorThreadLocked(PhysicsActor actor)
        {
            //m_log.Debug("RemoveAllJointsConnectedToActorThreadLocked: start");
 //           lock (OdeLock)
            {
                //m_log.Debug("RemoveAllJointsConnectedToActorThreadLocked: got lock");
                RemoveAllJointsConnectedToActor(actor);
            }
        }

        // normally called from within OnJointMoved, which is called from within a lock (OdeLock)
        public override Vector3 GetJointAnchor(PhysicsJoint joint)
        {
            Debug.Assert(joint.IsInPhysicsEngine);
            d.Vector3 pos = new d.Vector3();

            if (!(joint is OdePhysicsJoint))
            {
                DoJointErrorMessage(joint, "warning: non-ODE joint requesting anchor: " + joint.ObjectNameInScene);
            }
            else
            {
                OdePhysicsJoint odeJoint = (OdePhysicsJoint)joint;
                switch (odeJoint.Type)
                {
                    case PhysicsJointType.Ball:
                        d.JointGetBallAnchor(odeJoint.jointID, out pos);
                        break;
                    case PhysicsJointType.Hinge:
                        d.JointGetHingeAnchor(odeJoint.jointID, out pos);
                        break;
                }
            }
            return new Vector3(pos.X, pos.Y, pos.Z);
        }

        /// <summary>
        /// Get joint axis.
        /// </summary>
        /// <remarks>
        /// normally called from within OnJointMoved, which is called from within a lock (OdeLock)
        /// WARNING: ODE sometimes returns <0,0,0> as the joint axis! Therefore this function
        /// appears to be unreliable. Fortunately we can compute the joint axis ourselves by
        /// keeping track of the joint's original orientation relative to one of the involved bodies.
        /// </remarks>
        /// <param name="joint"></param>
        /// <returns></returns>
        public override Vector3 GetJointAxis(PhysicsJoint joint)
        {
            Debug.Assert(joint.IsInPhysicsEngine);
            d.Vector3 axis = new d.Vector3();

            if (!(joint is OdePhysicsJoint))
            {
                DoJointErrorMessage(joint, "warning: non-ODE joint requesting anchor: " + joint.ObjectNameInScene);
            }
            else
            {
                OdePhysicsJoint odeJoint = (OdePhysicsJoint)joint;
                switch (odeJoint.Type)
                {
                    case PhysicsJointType.Ball:
                        DoJointErrorMessage(joint, "warning - axis requested for ball joint: " + joint.ObjectNameInScene);
                        break;
                    case PhysicsJointType.Hinge:
                        d.JointGetHingeAxis(odeJoint.jointID, out axis);
                        break;
                }
            }
            return new Vector3(axis.X, axis.Y, axis.Z);
        }

        public void remActivePrim(OdePrim deactivatePrim)
        {
            lock (_activeprims)
            {
                _activeprims.Remove(deactivatePrim);
            }
        }

        public override void RemovePrim(PhysicsActor prim)
        {
            // As with all ODE physics operations, we don't remove the prim immediately but signal that it should be
            // removed in the next physics simulate pass.
            if (prim is OdePrim)
            {
//                lock (OdeLock)
                {
                    OdePrim p = (OdePrim)prim;
                    p.setPrimForRemoval();
                }
            }
        }
        /// <summary>
        /// This is called from within simulate but outside the locked portion
        /// We need to do our own locking here
        /// (Note: As of 20110801 this no longer appears to be true - this is being called within lock (odeLock) in
        /// Simulate() -- justincc).
        ///
        /// Essentially, we need to remove the prim from our space segment, whatever segment it's in.
        ///
        /// If there are no more prim in the segment, we need to empty (spacedestroy)the segment and reclaim memory
        /// that the space was using.
        /// </summary>
        /// <param name="prim"></param>
        public void RemovePrimThreadLocked(OdePrim prim)
        {
            //Console.WriteLine("RemovePrimThreadLocked " +  prim.m_primName);
            lock (prim)
            {
                RemoveCollisionEventReporting(prim);
                lock (_prims)
                    _prims.Remove(prim);

                if (SupportsNINJAJoints)
                {
                    RemoveAllJointsConnectedToActorThreadLocked(prim);
                }
            }

        }
        #endregion

        #region Space Separation Calculation

        /// <summary>
        /// Called when a static prim moves or becomes static
        /// Places the prim in a space one the static sub-spaces grid
        /// </summary>
        /// <param name="geom">the pointer to the geom that moved</param>
        /// <param name="pos">the position that the geom moved to</param>
        /// <param name="currentspace">a pointer to the space it was in before it was moved.</param>
        /// <returns>a pointer to the new space it's in</returns>
        public IntPtr MoveGeomToStaticSpace(IntPtr geom, Vector3 pos, IntPtr currentspace)
        {
            // moves a prim into another static sub-space or from another space into a static sub-space

            // Called ODEPrim so
            // it's already in locked space.

            if (geom == IntPtr.Zero) // shouldn't happen
                return IntPtr.Zero;

            // get the static sub-space for current position
            IntPtr newspace = calculateSpaceForGeom(pos);

            if (newspace == currentspace) // if we are there all done
                return newspace;

            // else remove it from its current space
            if (currentspace != IntPtr.Zero && d.SpaceQuery(currentspace, geom))
            {
                if (d.GeomIsSpace(currentspace))
                {
                    waitForSpaceUnlock(currentspace);
                    d.SpaceRemove(currentspace, geom);
                }
                else
                {
                    m_log.Info("[Physics]: Invalid or empty Space passed to 'MoveGeomToStaticSpace':" + currentspace +
                                   " Geom:" + geom);
                }
            }
            else // odd currentspace is null or doesn't contain the geom? lets try the geom ideia of current space
            {
                currentspace = d.GeomGetSpace(geom);
                if (currentspace != IntPtr.Zero)
                {
                    if (d.GeomIsSpace(currentspace))
                    {
                        waitForSpaceUnlock(currentspace);
                        d.SpaceRemove(currentspace, geom);
                    }
                }
            }

            // put the geom in the newspace
            waitForSpaceUnlock(newspace);
            d.SpaceAdd(newspace, geom);

            // let caller know this newspace
            return newspace;
        }

        /// <summary>
        /// Calculates the space the prim should be in by its position
        /// </summary>
        /// <param name="pos"></param>
        /// <returns>a pointer to the space. This could be a new space or reused space.</returns>
        public IntPtr calculateSpaceForGeom(Vector3 pos)
        {
            int x, y;
            x = (int)(pos.X * spacesPerMeter);
            if (x < 0)
                x = 0;
            else if (x > spaceGridMaxX)
                x = spaceGridMaxX;

            y = (int)(pos.Y * spacesPerMeter);
            if (y < 0)
                y = 0;
            else if (y >spaceGridMaxY)
                y = spaceGridMaxY;

            IntPtr tmpSpace = staticPrimspace[x, y];
            return tmpSpace;
        }
 
        #endregion

        /// <summary>
        /// Routine to figure out if we need to mesh this prim with our mesher
        /// </summary>
        /// <param name="pbs"></param>
        /// <returns></returns>
        public bool needsMeshing(PrimitiveBaseShape pbs)
        {
            // most of this is redundant now as the mesher will return null if it cant mesh a prim
            // but we still need to check for sculptie meshing being enabled so this is the most
            // convenient place to do it for now...

        //    //if (pbs.PathCurve == (byte)Primitive.PathCurve.Circle && pbs.ProfileCurve == (byte)Primitive.ProfileCurve.Circle && pbs.PathScaleY <= 0.75f)
        //    //m_log.Debug("needsMeshing: " + " pathCurve: " + pbs.PathCurve.ToString() + " profileCurve: " + pbs.ProfileCurve.ToString() + " pathScaleY: " + Primitive.UnpackPathScale(pbs.PathScaleY).ToString());
            int iPropertiesNotSupportedDefault = 0;

            if (pbs.SculptEntry)
            {
                if(!meshSculptedPrim)
                    return false;
            }

            // if it's a standard box or sphere with no cuts, hollows, twist or top shear, return false since ODE can use an internal representation for the prim
            if (!forceSimplePrimMeshing && !pbs.SculptEntry)
            {
                if ((pbs.ProfileShape == ProfileShape.Square && pbs.PathCurve == (byte)Extrusion.Straight)
                    || (pbs.ProfileShape == ProfileShape.HalfCircle && pbs.PathCurve == (byte)Extrusion.Curve1
                    && pbs.Scale.X == pbs.Scale.Y && pbs.Scale.Y == pbs.Scale.Z))
                {

                    if (pbs.ProfileBegin == 0 && pbs.ProfileEnd == 0
                        && pbs.ProfileHollow == 0
                        && pbs.PathTwist == 0 && pbs.PathTwistBegin == 0
                        && pbs.PathBegin == 0 && pbs.PathEnd == 0
                        && pbs.PathTaperX == 0 && pbs.PathTaperY == 0
                        && pbs.PathScaleX == 100 && pbs.PathScaleY == 100
                        && pbs.PathShearX == 0 && pbs.PathShearY == 0)
                    {
#if SPAM
                    m_log.Warn("NonMesh");
#endif
                        return false;
                    }
                }
            }

            //  following code doesn't give meshs to boxes and spheres ever
            // and it's odd..  so for now just return true if asked to force meshs
            // hopefully mesher will fail if doesn't suport so things still get basic boxes

            if (forceSimplePrimMeshing)
                return true;

            if (pbs.ProfileHollow != 0)
                iPropertiesNotSupportedDefault++;

            if ((pbs.PathBegin != 0) || pbs.PathEnd != 0)
                iPropertiesNotSupportedDefault++;

            if ((pbs.PathTwistBegin != 0) || (pbs.PathTwist != 0))
                iPropertiesNotSupportedDefault++; 

            if ((pbs.ProfileBegin != 0) || pbs.ProfileEnd != 0)
                iPropertiesNotSupportedDefault++;

            if ((pbs.PathScaleX != 100) || (pbs.PathScaleY != 100))
                iPropertiesNotSupportedDefault++;

            if ((pbs.PathShearX != 0) || (pbs.PathShearY != 0))
                iPropertiesNotSupportedDefault++;

            if (pbs.ProfileShape == ProfileShape.Circle && pbs.PathCurve == (byte)Extrusion.Straight)
                iPropertiesNotSupportedDefault++;

            if (pbs.ProfileShape == ProfileShape.HalfCircle && pbs.PathCurve == (byte)Extrusion.Curve1 && (pbs.Scale.X != pbs.Scale.Y || pbs.Scale.Y != pbs.Scale.Z || pbs.Scale.Z != pbs.Scale.X))
                iPropertiesNotSupportedDefault++;

            if (pbs.ProfileShape == ProfileShape.HalfCircle && pbs.PathCurve == (byte) Extrusion.Curve1)
                iPropertiesNotSupportedDefault++;

            // test for torus
            if ((pbs.ProfileCurve & 0x07) == (byte)ProfileShape.Square)
            {
                if (pbs.PathCurve == (byte)Extrusion.Curve1)
                {
                    iPropertiesNotSupportedDefault++;
                }
            }
            else if ((pbs.ProfileCurve & 0x07) == (byte)ProfileShape.Circle)
            {
                if (pbs.PathCurve == (byte)Extrusion.Straight)
                {
                    iPropertiesNotSupportedDefault++;
                }

                // ProfileCurve seems to combine hole shape and profile curve so we need to only compare against the lower 3 bits
                else if (pbs.PathCurve == (byte)Extrusion.Curve1)
                {
                    iPropertiesNotSupportedDefault++;
                }
            }
            else if ((pbs.ProfileCurve & 0x07) == (byte)ProfileShape.HalfCircle)
            {
                if (pbs.PathCurve == (byte)Extrusion.Curve1 || pbs.PathCurve == (byte)Extrusion.Curve2)
                {
                    iPropertiesNotSupportedDefault++;
                }
            }
            else if ((pbs.ProfileCurve & 0x07) == (byte)ProfileShape.EquilateralTriangle)
            {
                if (pbs.PathCurve == (byte)Extrusion.Straight)
                {
                    iPropertiesNotSupportedDefault++;
                }
                else if (pbs.PathCurve == (byte)Extrusion.Curve1)
                {
                    iPropertiesNotSupportedDefault++;
                }
            }

            if (pbs.SculptEntry && meshSculptedPrim)
                iPropertiesNotSupportedDefault++;

            if (iPropertiesNotSupportedDefault == 0)
            {
#if SPAM
                m_log.Warn("NonMesh");
#endif
                return false;
            }
#if SPAM
            m_log.Debug("Mesh");
#endif
            return true; 
        }

        public void AddChange(OdePrim prim, changes what, Object arg)
        {
            ODEchangeitem item = new ODEchangeitem();
            item.prim = prim;
            item.what = what;
            item.arg = arg;
            ChangesQueue.Enqueue(item);
        }

        /// <summary>
        /// Called to queue a change to a prim
        /// to use in place of old taint mechanism so changes do have a time sequence
        /// </summary>
        public void AddChange(OdeCharacter character, changes what, Object arg)
        {
            ODEchangeitem item = new ODEchangeitem();
            item.character = character;
            item.what = what;
            item.arg = arg;
            ChangesQueue.Enqueue(item);
        }

        /// <summary>
        /// Called after our prim properties are set Scale, position etc.
        /// We use this event queue like method to keep changes to the physical scene occuring in the threadlocked mutex
        /// This assures us that we have no race conditions
        /// </summary>
        /// <param name="prim"></param>
        public override void AddPhysicsActorTaint(PhysicsActor prim)
            {
            if (prim is OdePrim)
                {
/*                OdePrim taintedprim = ((OdePrim) prim);
                lock (_taintedPrimLock)
                    {
                    if (!(_taintedPrimH.Contains(taintedprim))) 
                        {
                        _taintedPrimH.Add(taintedprim);                    // HashSet for searching
                        _taintedPrimQ.Enqueue(taintedprim);                    // List for ordered readout
                        }
                    }
 */
                return;
                }
            else if (prim is OdeCharacter)
                {
                OdeCharacter taintedchar = ((OdeCharacter)prim);
                lock (_taintedCharacterLock)
                    {
                    if (!(_taintedCharacterH.Contains(taintedchar)))
                        {
                        _taintedCharacterH.Add(taintedchar);
                        _taintedCharacterQ.Enqueue(taintedchar);
                        if (taintedchar.bad)
                            m_log.DebugFormat("[PHYSICS]: Added BAD actor {0} to tainted actors", taintedchar.m_uuid);
                        }
                    }
                }
            }

        /// <summary>
        /// This is our main simulate loop
        /// It's thread locked by a Mutex in the scene.
        /// It holds Collisions, it instructs ODE to step through the physical reactions
        /// It moves the objects around in memory
        /// It calls the methods that report back to the object owners.. (scenepresence, SceneObjectGroup)
        /// </summary>
        /// <param name="timeStep"></param>
        /// <returns></returns>
        public override float Simulate(float timeStep)
        {
            if (framecount >= int.MaxValue)
                framecount = 0;

            framecount++;

            // acumulate time so we can reduce error
            step_time += timeStep;

            if (step_time < ODE_STEPSIZE)
                return 0;

            int curphysiteractions = m_physicsiterations;

            if (step_time >= m_SkipFramesAtms)
            {
                // if in trouble reduce step resolution
                curphysiteractions /= 2;
            }

            if (SupportsNINJAJoints)
            {
                DeleteRequestedJoints(); // this must be outside of the lock (OdeLock) to avoid deadlocks
                CreateRequestedJoints(); // this must be outside of the lock (OdeLock) to avoid deadlocks
            }

            int nodeframes = 0;

            lock (SimulationLock)
            {
                // adjust number of iterations per step
                try
                {
                    d.WorldSetQuickStepNumIterations(world, curphysiteractions);
                }
                catch (StackOverflowException)
                {
                    m_log.Error("[PHYSICS]: The operating system wasn't able to allocate enough memory for the simulation.  Restarting the sim.");
//                    ode.drelease(world);
                    base.TriggerPhysicsBasedRestart();
                }


                while (step_time > ODE_STEPSIZE && nodeframes < 10) //limit number of steps so we don't say here for ever
                {
                    try
                    {
                        // clear pointer/counter to contacts to pass into joints
                        m_global_contactcount = 0;

                        // do characters requested changes

                        OdeCharacter character;
                        int numtaints;
                        lock (_taintedCharacterLock)
                        {
                            numtaints = _taintedCharacterQ.Count;
                            //                            if (numtaints > 50)
                            //                                numtaints = 50;
                            while (numtaints > 0)
                            {
                                character = _taintedCharacterQ.Dequeue();
                                character.ProcessTaints(ODE_STEPSIZE);
                                _taintedCharacterH.Remove(character);
                                numtaints--;
                            }
                        }
                        // do other objects requested changes

                        ODEchangeitem item;
                        
                        if(ChangesQueue.Count >0)
                        {
                            int ttmpstart = Util.EnvironmentTickCount();
                            int ttmp;
                            int ttmp2;

                            while(ChangesQueue.Dequeue(out item))
                            {
                                if (item.prim != null)
                                {
                                    try
                                    {
                                        if (item.prim.DoAChange(item.what, item.arg))
                                            RemovePrimThreadLocked(item.prim);
                                    }
                                    catch { };
                                }
                                ttmp = Util.EnvironmentTickCountSubtract(ttmpstart);
                                if (ttmp > 20)
                                    break;
                            }

                            ttmp2 = Util.EnvironmentTickCountSubtract(ttmpstart);
                            if (ttmp2 > 50)
                                ttmp2 = 0;

                        }

                        if (SupportsNINJAJoints)
                                SimulatePendingNINJAJoints();

                        // Move characters
                        lock (_characters)
                        {
                            List<OdeCharacter> defects = new List<OdeCharacter>();
                            foreach (OdeCharacter actor in _characters)
                            {
                                if (actor != null)
                                    actor.Move(ODE_STEPSIZE, defects);
                            }
                            if (defects.Count != 0)
                            {
                                foreach (OdeCharacter defect in defects)
                                {
                                    RemoveCharacter(defect);
                                }
                            }
                        }

                        // Move other active objects
                        lock (_activeprims)
                        {
                            foreach (OdePrim aprim in _activeprims)
                            {
                                aprim.CollisionScore = 0;
                                aprim.IsColliding = false; // reset colision state. It will take 2 to cancel one
                                aprim.Move(ODE_STEPSIZE);
                            }
                        }

                        //if ((framecount % m_randomizeWater) == 0)
                        // randomizeWater(waterlevel);

                        m_rayCastManager.ProcessQueuedRequests();

                        collision_optimized(ODE_STEPSIZE);

                        lock (_collisionEventPrim)
                        {
                            foreach (PhysicsActor obj in _collisionEventPrim)
                            {
                                if (obj == null)
                                    continue;

                                switch ((ActorTypes)obj.PhysicsActorType)
                                {
                                    case ActorTypes.Agent:
                                        OdeCharacter cobj = (OdeCharacter)obj;
                                        cobj.AddCollisionFrameTime((int)(ODE_STEPSIZE*1000.0f));
                                        cobj.SendCollisions();
                                        break;

                                    case ActorTypes.Prim:
                                        OdePrim pobj = (OdePrim)obj;
                                        pobj.SendCollisions();
                                        break;
                                }
                            }
                        }

                        d.WorldQuickStep(world, ODE_STEPSIZE);
                        d.JointGroupEmpty(contactgroup);
                        //ode.dunlock(world);
                    }
                    catch (Exception e)
                    {
                        m_log.ErrorFormat("[PHYSICS]: {0}, {1}, {2}", e.Message, e.TargetSite, e);
//                        ode.dunlock(world);
                    }

                    step_time -= ODE_STEPSIZE;
                    nodeframes++;
                }

                lock (_characters)
                {
                    foreach (OdeCharacter actor in _characters)
                    {
                        if (actor != null)
                        {
                            if (actor.bad)
                                m_log.WarnFormat("[PHYSICS]: BAD Actor {0} in _characters list was not removed?", actor.m_uuid);

                            actor.UpdatePositionAndVelocity();
                        }
                    }
                }

                lock (_badCharacter)
                {
                    if (_badCharacter.Count > 0)
                    {
                        foreach (OdeCharacter chr in _badCharacter)
                        {
                            RemoveCharacter(chr);
                        }

                        _badCharacter.Clear();
                    }
                }

                lock (_activeprims)
                {
                    {
                        foreach (OdePrim actor in _activeprims)
                        {
                            if (actor.IsPhysical)
                            {
                                actor.UpdatePositionAndVelocity((float)nodeframes * ODE_STEPSIZE);

                                if (SupportsNINJAJoints)
                                    SimulateActorPendingJoints(actor);
                            }
                        }
                    }
                }
                //DumpJointInfo();

                // Finished with all sim stepping. If requested, dump world state to file for debugging.
                // TODO: This call to the export function is already inside lock (OdeLock) - but is an extra lock needed?
                // TODO: This overwrites all dump files in-place. Should this be a growing logfile, or separate snapshots?
                if (physics_logging && (physics_logging_interval > 0) && (framecount % physics_logging_interval == 0))
                {
                    string fname = "state-" + world.ToString() + ".DIF"; // give each physics world a separate filename
                    string prefix = "world" + world.ToString(); // prefix for variable names in exported .DIF file

                    if (physics_logging_append_existing_logfile)
                    {
                        string header = "-------------- START OF PHYSICS FRAME " + framecount.ToString() + " --------------";
                        TextWriter fwriter = File.AppendText(fname);
                        fwriter.WriteLine(header);
                        fwriter.Close();
                    }

                    d.WorldExportDIF(world, fname, physics_logging_append_existing_logfile, prefix);
                }

                // think time dilation is not a physics issue alone..  but ok let's fake something
                if (step_time < ODE_STEPSIZE) // we did the required loops
                    m_timeDilation = 1.0f;
                else
                { // we didn't forget the lost ones and let user know something
                    m_timeDilation = 1 - step_time / timeStep;
                    if (m_timeDilation < 0)
                        m_timeDilation = 0;
                    step_time = 0;
                }
            }

            return nodeframes * ODE_STEPSIZE;
        }

        /// <summary>
        /// Simulate pending NINJA joints.
        /// </summary>
        /// <remarks>
        /// Called by the main Simulate() loop if NINJA joints are active.  Should not be called from anywhere else.
        /// </remarks>
        protected void SimulatePendingNINJAJoints()
        {
            // Create pending joints, if possible

            // joints can only be processed after ALL bodies are processed (and exist in ODE), since creating
            // a joint requires specifying the body id of both involved bodies
            if (pendingJoints.Count > 0)
            {
                List<PhysicsJoint> successfullyProcessedPendingJoints = new List<PhysicsJoint>();
                //DoJointErrorMessage(joints_connecting_actor, "taint: " + pendingJoints.Count + " pending joints");
                foreach (PhysicsJoint joint in pendingJoints)
                {
                    //DoJointErrorMessage(joint, "taint: time to create joint with parms: " + joint.RawParams);
                    string[] jointParams = joint.RawParams.Split(" ".ToCharArray(), System.StringSplitOptions.RemoveEmptyEntries);
                    List<IntPtr> jointBodies = new List<IntPtr>();
                    bool allJointBodiesAreReady = true;
                    foreach (string jointParam in jointParams)
                    {
                        if (jointParam == "NULL")
                        {
                            //DoJointErrorMessage(joint, "attaching NULL joint to world");
                            jointBodies.Add(IntPtr.Zero);
                        }
                        else
                        {
                            //DoJointErrorMessage(joint, "looking for prim name: " + jointParam);
                            bool foundPrim = false;
                            lock (_prims)
                            {
                                foreach (OdePrim prim in _prims) // FIXME: inefficient
                                {
                                    if (prim.SOPName == jointParam)
                                    {
                                        //DoJointErrorMessage(joint, "found for prim name: " + jointParam);
                                        if (prim.IsPhysical && prim.Body != IntPtr.Zero)
                                        {
                                            jointBodies.Add(prim.Body);
                                            foundPrim = true;
                                            break;
                                        }
                                        else
                                        {
                                            DoJointErrorMessage(joint, "prim name " + jointParam +
                                                " exists but is not (yet) physical; deferring joint creation. " +
                                                "IsPhysical property is " + prim.IsPhysical +
                                                " and body is " + prim.Body);
                                            foundPrim = false;
                                            break;
                                        }
                                    }
                                }
                            }
                            if (foundPrim)
                            {
                                // all is fine
                            }
                            else
                            {
                                allJointBodiesAreReady = false;
                                break;
                            }
                        }
                    }

                    if (allJointBodiesAreReady)
                    {
                        //DoJointErrorMessage(joint, "allJointBodiesAreReady for " + joint.ObjectNameInScene + " with parms " + joint.RawParams);
                        if (jointBodies[0] == jointBodies[1])
                        {
                            DoJointErrorMessage(joint, "ERROR: joint cannot be created; the joint bodies are the same, body1==body2. Raw body is " + jointBodies[0] + ". raw parms: " + joint.RawParams);
                        }
                        else
                        {
                            switch (joint.Type)
                            {
                                case PhysicsJointType.Ball:
                                    {
                                        IntPtr odeJoint;
                                        //DoJointErrorMessage(joint, "ODE creating ball joint ");
                                        odeJoint = d.JointCreateBall(world, IntPtr.Zero);
                                        //DoJointErrorMessage(joint, "ODE attaching ball joint: " + odeJoint + " with b1:" + jointBodies[0] + " b2:" + jointBodies[1]);
                                        d.JointAttach(odeJoint, jointBodies[0], jointBodies[1]);
                                        //DoJointErrorMessage(joint, "ODE setting ball anchor: " + odeJoint + " to vec:" + joint.Position);
                                        d.JointSetBallAnchor(odeJoint,
                                                            joint.Position.X,
                                                            joint.Position.Y,
                                                            joint.Position.Z);
                                        //DoJointErrorMessage(joint, "ODE joint setting OK");
                                        //DoJointErrorMessage(joint, "The ball joint's bodies are here: b0: ");
                                        //DoJointErrorMessage(joint, "" + (jointBodies[0] != IntPtr.Zero ? "" + d.BodyGetPosition(jointBodies[0]) : "fixed environment"));
                                        //DoJointErrorMessage(joint, "The ball joint's bodies are here: b1: ");
                                        //DoJointErrorMessage(joint, "" + (jointBodies[1] != IntPtr.Zero ? "" + d.BodyGetPosition(jointBodies[1]) : "fixed environment"));

                                        if (joint is OdePhysicsJoint)
                                        {
                                            ((OdePhysicsJoint)joint).jointID = odeJoint;
                                        }
                                        else
                                        {
                                            DoJointErrorMessage(joint, "WARNING: non-ode joint in ODE!");
                                        }
                                    }
                                    break;
                                case PhysicsJointType.Hinge:
                                    {
                                        IntPtr odeJoint;
                                        //DoJointErrorMessage(joint, "ODE creating hinge joint ");
                                        odeJoint = d.JointCreateHinge(world, IntPtr.Zero);
                                        //DoJointErrorMessage(joint, "ODE attaching hinge joint: " + odeJoint + " with b1:" + jointBodies[0] + " b2:" + jointBodies[1]);
                                        d.JointAttach(odeJoint, jointBodies[0], jointBodies[1]);
                                        //DoJointErrorMessage(joint, "ODE setting hinge anchor: " + odeJoint + " to vec:" + joint.Position);
                                        d.JointSetHingeAnchor(odeJoint,
                                                              joint.Position.X,
                                                              joint.Position.Y,
                                                              joint.Position.Z);
                                        // We use the orientation of the x-axis of the joint's coordinate frame
                                        // as the axis for the hinge.

                                        // Therefore, we must get the joint's coordinate frame based on the
                                        // joint.Rotation field, which originates from the orientation of the 
                                        // joint's proxy object in the scene.

                                        // The joint's coordinate frame is defined as the transformation matrix
                                        // that converts a vector from joint-local coordinates into world coordinates.
                                        // World coordinates are defined as the XYZ coordinate system of the sim,
                                        // as shown in the top status-bar of the viewer.

                                        // Once we have the joint's coordinate frame, we extract its X axis (AtAxis)
                                        // and use that as the hinge axis.

                                        //joint.Rotation.Normalize();
                                        Matrix4 proxyFrame = Matrix4.CreateFromQuaternion(joint.Rotation);

                                        // Now extract the X axis of the joint's coordinate frame.

                                        // Do not try to use proxyFrame.AtAxis or you will become mired in the
                                        // tar pit of transposed, inverted, and generally messed-up orientations.
                                        // (In other words, Matrix4.AtAxis() is borked.)
                                        // Vector3 jointAxis = proxyFrame.AtAxis; <--- this path leadeth to madness

                                        // Instead, compute the X axis of the coordinate frame by transforming
                                        // the (1,0,0) vector. At least that works.

                                        //m_log.Debug("PHY: making axis: complete matrix is " + proxyFrame);
                                        Vector3 jointAxis = Vector3.Transform(Vector3.UnitX, proxyFrame);
                                        //m_log.Debug("PHY: making axis: hinge joint axis is " + jointAxis);
                                        //DoJointErrorMessage(joint, "ODE setting hinge axis: " + odeJoint + " to vec:" + jointAxis);
                                        d.JointSetHingeAxis(odeJoint,
                                                            jointAxis.X,
                                                            jointAxis.Y,
                                                            jointAxis.Z);
                                        //d.JointSetHingeParam(odeJoint, (int)dParam.CFM, 0.1f);
                                        if (joint is OdePhysicsJoint)
                                        {
                                            ((OdePhysicsJoint)joint).jointID = odeJoint;
                                        }
                                        else
                                        {
                                            DoJointErrorMessage(joint, "WARNING: non-ode joint in ODE!");
                                        }
                                    }
                                    break;
                            }
                            successfullyProcessedPendingJoints.Add(joint);
                        }
                    }
                    else
                    {
                        DoJointErrorMessage(joint, "joint could not yet be created; still pending");
                    }
                }

                foreach (PhysicsJoint successfullyProcessedJoint in successfullyProcessedPendingJoints)
                {
                    //DoJointErrorMessage(successfullyProcessedJoint, "finalizing succesfully procsssed joint " + successfullyProcessedJoint.ObjectNameInScene + " parms " + successfullyProcessedJoint.RawParams);
                    //DoJointErrorMessage(successfullyProcessedJoint, "removing from pending");
                    InternalRemovePendingJoint(successfullyProcessedJoint);
                    //DoJointErrorMessage(successfullyProcessedJoint, "adding to active");
                    InternalAddActiveJoint(successfullyProcessedJoint);
                    //DoJointErrorMessage(successfullyProcessedJoint, "done");
                }
            }
        }

        /// <summary>
        /// Simulate the joint proxies of a NINJA actor.
        /// </summary>
        /// <remarks>
        /// Called as part of the Simulate() loop if NINJA physics is active.  Must only be called from there.
        /// </remarks>
        /// <param name="actor"></param>
        protected void SimulateActorPendingJoints(OdePrim actor)
        {
            // If an actor moved, move its joint proxy objects as well.
            // There seems to be an event PhysicsActor.OnPositionUpdate that could be used
            // for this purpose but it is never called! So we just do the joint
            // movement code here.

            if (actor.SOPName != null &&
                joints_connecting_actor.ContainsKey(actor.SOPName) &&
                joints_connecting_actor[actor.SOPName] != null &&
                joints_connecting_actor[actor.SOPName].Count > 0)
            {
                foreach (PhysicsJoint affectedJoint in joints_connecting_actor[actor.SOPName])
                {
                    if (affectedJoint.IsInPhysicsEngine)
                    {
                        DoJointMoved(affectedJoint);
                    }
                    else
                    {
                        DoJointErrorMessage(affectedJoint, "a body connected to a joint was moved, but the joint doesn't exist yet! this will lead to joint error. joint was: " + affectedJoint.ObjectNameInScene + " parms:" + affectedJoint.RawParams);
                    }
                }
            }
        }

        public override void GetResults()
        {
        }

        public override bool IsThreaded
        {
            // for now we won't be multithreaded
            get { return (false); }
        }

        #region ODE Specific Terrain Fixes
        public float[] ResizeTerrain512NearestNeighbour(float[] heightMap)
        {
            float[] returnarr = new float[262144];
            float[,] resultarr = new float[(int)WorldExtents.X, (int)WorldExtents.Y];

            // Filling out the array into its multi-dimensional components
            for (int y = 0; y < WorldExtents.Y; y++)
            {
                for (int x = 0; x < WorldExtents.X; x++)
                {
                    resultarr[y, x] = heightMap[y * (int)WorldExtents.Y + x];
                }
            }

            // Resize using Nearest Neighbour

            // This particular way is quick but it only works on a multiple of the original

            // The idea behind this method can be described with the following diagrams
            // second pass and third pass happen in the same loop really..  just separated
            // them to show what this does.

            // First Pass
            // ResultArr:
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1

            // Second Pass
            // ResultArr2:
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,

            // Third pass fills in the blanks
            // ResultArr2:
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1

            // X,Y = .
            // X+1,y = ^
            // X,Y+1 = *
            // X+1,Y+1 = #

            // Filling in like this;
            // .*
            // ^#
            // 1st .
            // 2nd *
            // 3rd ^
            // 4th #
            // on single loop.

            float[,] resultarr2 = new float[512, 512];
            for (int y = 0; y < WorldExtents.Y; y++)
            {
                for (int x = 0; x < WorldExtents.X; x++)
                {
                    resultarr2[y * 2, x * 2] = resultarr[y, x];

                    if (y < WorldExtents.Y)
                    {
                        resultarr2[(y * 2) + 1, x * 2] = resultarr[y, x];
                    }
                    if (x < WorldExtents.X)
                    {
                        resultarr2[y * 2, (x * 2) + 1] = resultarr[y, x];
                    }
                    if (x < WorldExtents.X && y < WorldExtents.Y)
                    {
                        resultarr2[(y * 2) + 1, (x * 2) + 1] = resultarr[y, x];
                    }
                }
            }

            //Flatten out the array
            int i = 0;
            for (int y = 0; y < 512; y++)
            {
                for (int x = 0; x < 512; x++)
                {
                    if (resultarr2[y, x] <= 0)
                        returnarr[i] = 0.0000001f;
                    else
                        returnarr[i] = resultarr2[y, x];

                    i++;
                }
            }

            return returnarr;
        }

        public float[] ResizeTerrain512Interpolation(float[] heightMap)
        {
            float[] returnarr = new float[262144];
            float[,] resultarr = new float[512,512];

            // Filling out the array into its multi-dimensional components
            for (int y = 0; y < 256; y++)
            {
                for (int x = 0; x < 256; x++)
                {
                    resultarr[y, x] = heightMap[y * 256 + x];
                }
            }

            // Resize using interpolation

            // This particular way is quick but it only works on a multiple of the original

            // The idea behind this method can be described with the following diagrams
            // second pass and third pass happen in the same loop really..  just separated
            // them to show what this does.

            // First Pass
            // ResultArr:
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1
            // 1,1,1,1,1,1

            // Second Pass
            // ResultArr2:
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,
            // ,,,,,,,,,,
            // 1,,1,,1,,1,,1,,1,

            // Third pass fills in the blanks
            // ResultArr2:
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1
            // 1,1,1,1,1,1,1,1,1,1,1,1

            // X,Y = .
            // X+1,y = ^
            // X,Y+1 = *
            // X+1,Y+1 = #

            // Filling in like this;
            // .*
            // ^#
            // 1st .
            // 2nd *
            // 3rd ^
            // 4th #
            // on single loop.

            float[,] resultarr2 = new float[512,512];
            for (int y = 0; y < (int)Constants.RegionSize; y++)
            {
                for (int x = 0; x < (int)Constants.RegionSize; x++)
                {
                    resultarr2[y*2, x*2] = resultarr[y, x];

                    if (y < (int)Constants.RegionSize)
                    {
                        if (y + 1 < (int)Constants.RegionSize)
                        {
                            if (x + 1 < (int)Constants.RegionSize)
                            {
                                resultarr2[(y*2) + 1, x*2] = ((resultarr[y, x] + resultarr[y + 1, x] +
                                                               resultarr[y, x + 1] + resultarr[y + 1, x + 1])/4);
                            }
                            else
                            {
                                resultarr2[(y*2) + 1, x*2] = ((resultarr[y, x] + resultarr[y + 1, x])/2);
                            }
                        }
                        else
                        {
                            resultarr2[(y*2) + 1, x*2] = resultarr[y, x];
                        }
                    }
                    if (x < (int)Constants.RegionSize)
                    {
                        if (x + 1 < (int)Constants.RegionSize)
                        {
                            if (y + 1 < (int)Constants.RegionSize)
                            {
                                resultarr2[y*2, (x*2) + 1] = ((resultarr[y, x] + resultarr[y + 1, x] +
                                                               resultarr[y, x + 1] + resultarr[y + 1, x + 1])/4);
                            }
                            else
                            {
                                resultarr2[y*2, (x*2) + 1] = ((resultarr[y, x] + resultarr[y, x + 1])/2);
                            }
                        }
                        else
                        {
                            resultarr2[y*2, (x*2) + 1] = resultarr[y, x];
                        }
                    }
                    if (x < (int)Constants.RegionSize && y < (int)Constants.RegionSize)
                    {
                        if ((x + 1 < (int)Constants.RegionSize) && (y + 1 < (int)Constants.RegionSize))
                        {
                            resultarr2[(y*2) + 1, (x*2) + 1] = ((resultarr[y, x] + resultarr[y + 1, x] +
                                                                 resultarr[y, x + 1] + resultarr[y + 1, x + 1])/4);
                        }
                        else
                        {
                            resultarr2[(y*2) + 1, (x*2) + 1] = resultarr[y, x];
                        }
                    }
                }
            }
            //Flatten out the array
            int i = 0;
            for (int y = 0; y < 512; y++)
            {
                for (int x = 0; x < 512; x++)
                {
                    if (Single.IsNaN(resultarr2[y, x]) || Single.IsInfinity(resultarr2[y, x]))
                    {
                        m_log.Warn("[PHYSICS]: Non finite heightfield element detected.  Setting it to 0");
                        resultarr2[y, x] = 0;
                    }
                    returnarr[i] = resultarr2[y, x];
                    i++;
                }
            }

            return returnarr;
        }

        #endregion

        public override void SetTerrain(float[] heightMap)
        {
            if (m_worldOffset != Vector3.Zero && m_parentScene != null)
            {
                if (m_parentScene is OdeScene)
                {
                    ((OdeScene)m_parentScene).SetTerrain(heightMap, m_worldOffset);
                }
            }
            else
            {
                SetTerrain(heightMap, m_worldOffset);
            }
        }

        public override void CombineTerrain(float[] heightMap, Vector3 pOffset)
        {
            SetTerrain(heightMap, pOffset);
        }

        public void SetTerrain(float[] heightMap, Vector3 pOffset)
            {

            float[] _heightmap;
            _heightmap = new float[(((int)Constants.RegionSize + 2) * ((int)Constants.RegionSize + 2))];

            uint heightmapWidth = Constants.RegionSize + 2;
            uint heightmapHeight = Constants.RegionSize + 2;

            uint heightmapWidthSamples;

            uint heightmapHeightSamples;
 
            heightmapWidthSamples = (uint)Constants.RegionSize + 2;
            heightmapHeightSamples = (uint)Constants.RegionSize + 2;

            const float scale = 1.0f;
            const float offset = 0.0f;
            const float thickness = 10f;
            const int wrap = 0;

            int regionsize = (int) Constants.RegionSize + 2;
 
            float hfmin = float.MaxValue;
            float hfmax = float.MinValue;
            float val;
            int xx;
            int yy;

            int maxXXYY = regionsize - 3;
            // flipping map adding one margin all around so things don't fall in edges

            int xt = 0;
            xx = 0;

            for (int x = 0; x < heightmapWidthSamples; x++)
            {
                if (x > 1 && xx < maxXXYY)
                    xx++;
                yy = 0;
                for (int y = 0; y < heightmapHeightSamples; y++)
                {
                    if (y > 1 && y < maxXXYY)
                        yy += (int)Constants.RegionSize;

                    val = heightMap[yy + xx];
                    _heightmap[xt + y] = val;

                    if (hfmin > val)
                        hfmin = val;
                    if (hfmax < val)
                        hfmax = val;

                }

                xt += regionsize;
            }
            lock (OdeLock)
            {
                IntPtr GroundGeom = IntPtr.Zero;
                if (RegionTerrain.TryGetValue(pOffset, out GroundGeom))
                {
                    RegionTerrain.Remove(pOffset);
                    if (GroundGeom != IntPtr.Zero)
                    {
                        if (TerrainHeightFieldHeights.ContainsKey(GroundGeom))
                            {
                            TerrainHeightFieldHeightsHandlers[GroundGeom].Free();
                            TerrainHeightFieldHeightsHandlers.Remove(GroundGeom);
                            TerrainHeightFieldHeights.Remove(GroundGeom);
                            }
                        d.SpaceRemove(StaticSpace, GroundGeom);
                        d.GeomDestroy(GroundGeom);
                    }
                }
                IntPtr HeightmapData = d.GeomHeightfieldDataCreate();

                GCHandle _heightmaphandler = GCHandle.Alloc(_heightmap, GCHandleType.Pinned);

                d.GeomHeightfieldDataBuildSingle(HeightmapData, _heightmaphandler.AddrOfPinnedObject(), 0, heightmapWidth , heightmapHeight,
                                                 (int)heightmapWidthSamples, (int)heightmapHeightSamples, scale,
                                                 offset, thickness, wrap);

                d.GeomHeightfieldDataSetBounds(HeightmapData, hfmin - 1, hfmax + 1);
                GroundGeom = d.CreateHeightfield(StaticSpace, HeightmapData, 1);
                if (GroundGeom != IntPtr.Zero)
                {
                    d.GeomSetCategoryBits(GroundGeom, (int)(CollisionCategories.Land));
                    d.GeomSetCollideBits(GroundGeom, (int)(CollisionCategories.Space));

                }
                geom_name_map[GroundGeom] = "Terrain";

                d.Matrix3 R = new d.Matrix3();

                Quaternion q1 = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 1.5707f);
                Quaternion q2 = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 1.5707f);
                

                q1 = q1 * q2;
                
                Vector3 v3;
                float angle;
                q1.GetAxisAngle(out v3, out angle);

                d.RFromAxisAndAngle(out R, v3.X, v3.Y, v3.Z, angle);
                d.GeomSetRotation(GroundGeom, ref R);
                d.GeomSetPosition(GroundGeom, (pOffset.X + ((int)Constants.RegionSize * 0.5f)), (pOffset.Y + ((int)Constants.RegionSize * 0.5f)), 0);
                IntPtr testGround = IntPtr.Zero;
                if (RegionTerrain.TryGetValue(pOffset, out testGround))
                {
                    RegionTerrain.Remove(pOffset);
                }
                RegionTerrain.Add(pOffset, GroundGeom, GroundGeom);
//                TerrainHeightFieldHeights.Add(GroundGeom, ODElandMap);
                TerrainHeightFieldHeights.Add(GroundGeom, _heightmap);
                TerrainHeightFieldHeightsHandlers.Add(GroundGeom, _heightmaphandler);
               
            }
        }

        public override void DeleteTerrain()
        {
        }

        public float GetWaterLevel()
        {
            return waterlevel;
        }

        public override bool SupportsCombining()
        {
            return true;
        }
/*
        public override void UnCombine(PhysicsScene pScene)
        {
            IntPtr localGround = IntPtr.Zero;
//            float[] localHeightfield;
            bool proceed = false;
            List<IntPtr> geomDestroyList = new List<IntPtr>();

            lock (OdeLock)
            {
                if (RegionTerrain.TryGetValue(Vector3.Zero, out localGround))
                {
                    foreach (IntPtr geom in TerrainHeightFieldHeights.Keys)
                    {
                        if (geom == localGround)
                        {
//                            localHeightfield = TerrainHeightFieldHeights[geom];
                            proceed = true;
                        }
                        else
                        {
                            geomDestroyList.Add(geom);
                        }
                    }

                    if (proceed)
                    {
                        m_worldOffset = Vector3.Zero;
                        WorldExtents = new Vector2((int)Constants.RegionSize, (int)Constants.RegionSize);
                        m_parentScene = null;

                        foreach (IntPtr g in geomDestroyList)
                        {
                            // removingHeightField needs to be done or the garbage collector will
                            // collect the terrain data before we tell ODE to destroy it causing 
                            // memory corruption
                            if (TerrainHeightFieldHeights.ContainsKey(g))
                            {
//                                float[] removingHeightField = TerrainHeightFieldHeights[g];
                                TerrainHeightFieldHeights.Remove(g);

                                if (RegionTerrain.ContainsKey(g))
                                {
                                    RegionTerrain.Remove(g);
                                }

                                d.GeomDestroy(g);
                                //removingHeightField = new float[0];
                            }
                        }

                    }
                    else
                    {
                        m_log.Warn("[PHYSICS]: Couldn't proceed with UnCombine.  Region has inconsistant data.");
                    }
                }
            }
        }
*/
        public override void SetWaterLevel(float baseheight)
        {
            waterlevel = baseheight;
            randomizeWater(waterlevel);
        }

        public void randomizeWater(float baseheight)
        {
            const uint heightmapWidth = m_regionWidth + 2;
            const uint heightmapHeight = m_regionHeight + 2;
            const uint heightmapWidthSamples = m_regionWidth + 2;
            const uint heightmapHeightSamples = m_regionHeight + 2;
            const float scale = 1.0f;
            const float offset = 0.0f;
            const float thickness = 2.9f;
            const int wrap = 0;

            for (int i = 0; i < (258 * 258); i++)
            {
                _watermap[i] = (baseheight-0.1f) + ((float)fluidRandomizer.Next(1,9) / 10f);
               // m_log.Info((baseheight - 0.1f) + ((float)fluidRandomizer.Next(1, 9) / 10f));
            }

            lock (OdeLock)
            {
                if (WaterGeom != IntPtr.Zero)
                {
                d.SpaceRemove(StaticSpace, WaterGeom);
                }
                IntPtr HeightmapData = d.GeomHeightfieldDataCreate();
                d.GeomHeightfieldDataBuildSingle(HeightmapData, _watermap, 0, heightmapWidth, heightmapHeight,
                                                 (int)heightmapWidthSamples, (int)heightmapHeightSamples, scale,
                                                 offset, thickness, wrap);
                d.GeomHeightfieldDataSetBounds(HeightmapData, m_regionWidth, m_regionHeight);
                WaterGeom = d.CreateHeightfield(StaticSpace, HeightmapData, 1);
                if (WaterGeom != IntPtr.Zero)
                {
                    d.GeomSetCategoryBits(WaterGeom, (int)(CollisionCategories.Water));
                    d.GeomSetCollideBits(WaterGeom, (int)(CollisionCategories.Space));

                }
                geom_name_map[WaterGeom] = "Water";

                d.Matrix3 R = new d.Matrix3();

                Quaternion q1 = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 1.5707f);
                Quaternion q2 = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 1.5707f);

                q1 = q1 * q2;
                Vector3 v3;
                float angle;
                q1.GetAxisAngle(out v3, out angle);

                d.RFromAxisAndAngle(out R, v3.X, v3.Y, v3.Z, angle);
                d.GeomSetRotation(WaterGeom, ref R);
                d.GeomSetPosition(WaterGeom, 128, 128, 0);
                
            }

        }

        public override void Dispose()
        {
            m_rayCastManager.Dispose();
            m_rayCastManager = null;

            lock (OdeLock)
            {
                lock (_prims)
                {
                    foreach (OdePrim prm in _prims)
                    {
                        RemovePrim(prm);
                    }
                }

                if (ContactgeomsArray != IntPtr.Zero)
                    Marshal.FreeHGlobal(ContactgeomsArray);
                if (GlobalContactsArray != IntPtr.Zero)
                    Marshal.FreeHGlobal(GlobalContactsArray);

                d.WorldDestroy(world);
                //d.CloseODE();
            }
        }

        public override Dictionary<uint, float> GetTopColliders()
        {
            Dictionary<uint, float> returncolliders = new Dictionary<uint, float>();
            int cnt = 0;
            lock (_prims)
            {
                foreach (OdePrim prm in _prims)
                {
                    if (prm.CollisionScore > 0)
                    {
                        returncolliders.Add(prm.m_localID, prm.CollisionScore);
                        cnt++;
                        prm.CollisionScore = 0f;
                        if (cnt > 25)
                        {
                            break;
                        }
                    }
                }
            }
            return returncolliders;
        }

        public override bool SupportsRayCast()
        {
            return true;
        }

        public override void RaycastWorld(Vector3 position, Vector3 direction, float length, RaycastCallback retMethod)
        {
            if (retMethod != null)
            {
                m_rayCastManager.QueueRequest(position, direction, length, retMethod);
            }
        }

        public override void RaycastWorld(Vector3 position, Vector3 direction, float length, int Count, RayCallback retMethod)
        {
            if (retMethod != null)
            {
                m_rayCastManager.QueueRequest(position, direction, length, Count, retMethod);
            }
        }

        // don't like this
        public override List<ContactResult> RaycastWorld(Vector3 position, Vector3 direction, float length, int Count)
        {
            ContactResult[] ourResults = null;
            RayCallback retMethod = delegate(List<ContactResult> results)
            {
                ourResults = new ContactResult[results.Count];
                results.CopyTo(ourResults, 0);
            };
            int waitTime = 0;
            m_rayCastManager.QueueRequest(position, direction, length, Count, retMethod);
            while (ourResults == null && waitTime < 1000)
            {
                Thread.Sleep(1);
                waitTime++;
            }
            if (ourResults == null)
                return new List<ContactResult>();
            return new List<ContactResult>(ourResults);
        }

        public override void RaycastActor(PhysicsActor actor, Vector3 position, Vector3 direction, float length, RaycastCallback retMethod)
        {
            if (retMethod != null && actor !=null)
            {
                IntPtr geom;
                if (actor is OdePrim)
                    geom = ((OdePrim)actor).prim_geom;
                else if (actor is OdeCharacter)
                    geom = ((OdePrim)actor).prim_geom;
                else
                    return;
                if (geom == IntPtr.Zero)
                    return;
                m_rayCastManager.QueueRequest(geom, position, direction, length, retMethod);
            }
        }

        public override void RaycastActor(PhysicsActor actor, Vector3 position, Vector3 direction, float length, int Count, RayCallback retMethod)
        {
            if (retMethod != null && actor != null)
            {
                IntPtr geom;
                if (actor is OdePrim)
                    geom = ((OdePrim)actor).prim_geom;
                else if (actor is OdeCharacter)
                    geom = ((OdePrim)actor).prim_geom;
                else
                    return;
                if (geom == IntPtr.Zero)
                    return;

                m_rayCastManager.QueueRequest(geom,position, direction, length, Count, retMethod);
            }
        }

        // don't like this
        public override List<ContactResult> RaycastActor(PhysicsActor actor, Vector3 position, Vector3 direction, float length, int Count)
        {
            if (actor != null)
            {
                IntPtr geom;
                if (actor is OdePrim)
                    geom = ((OdePrim)actor).prim_geom;
                else if (actor is OdeCharacter)
                    geom = ((OdePrim)actor).prim_geom;
                else
                    return new List<ContactResult>();
                if (geom == IntPtr.Zero)
                    return new List<ContactResult>();

                ContactResult[] ourResults = null;
                RayCallback retMethod = delegate(List<ContactResult> results)
                {
                    ourResults = new ContactResult[results.Count];
                    results.CopyTo(ourResults, 0);
                };
                int waitTime = 0;
                m_rayCastManager.QueueRequest(geom,position, direction, length, Count, retMethod);
                while (ourResults == null && waitTime < 1000)
                {
                    Thread.Sleep(1);
                    waitTime++;
                }
                if (ourResults == null)
                    return new List<ContactResult>();
                return new List<ContactResult>(ourResults);
            }
            return new List<ContactResult>();
        }

#if USE_DRAWSTUFF
        // Keyboard callback
        public void command(int cmd)
        {
            IntPtr geom;
            d.Mass mass;
            d.Vector3 sides = new d.Vector3(d.RandReal() * 0.5f + 0.1f, d.RandReal() * 0.5f + 0.1f, d.RandReal() * 0.5f + 0.1f);

            

            Char ch = Char.ToLower((Char)cmd);
            switch ((Char)ch)
            {
                case 'w':
                    try
                    {
                        Vector3 rotate = (new Vector3(1, 0, 0) * Quaternion.CreateFromEulers(hpr.Z * Utils.DEG_TO_RAD, hpr.Y * Utils.DEG_TO_RAD, hpr.X * Utils.DEG_TO_RAD));

                        xyz.X += rotate.X; xyz.Y += rotate.Y; xyz.Z += rotate.Z;
                        ds.SetViewpoint(ref xyz, ref hpr);
                    }
                    catch (ArgumentException)
                    { hpr.X = 0; }
                    break;

                case 'a':
                    hpr.X++;
                    ds.SetViewpoint(ref xyz, ref hpr);
                    break;

                case 's':
                    try
                    {
                        Vector3 rotate2 = (new Vector3(-1, 0, 0) * Quaternion.CreateFromEulers(hpr.Z * Utils.DEG_TO_RAD, hpr.Y * Utils.DEG_TO_RAD, hpr.X * Utils.DEG_TO_RAD));

                        xyz.X += rotate2.X; xyz.Y += rotate2.Y; xyz.Z += rotate2.Z;
                        ds.SetViewpoint(ref xyz, ref hpr);
                    }
                    catch (ArgumentException)
                    { hpr.X = 0; }
                    break;
                case 'd':
                    hpr.X--;
                    ds.SetViewpoint(ref xyz, ref hpr);
                    break;
                case 'r':
                    xyz.Z++;
                    ds.SetViewpoint(ref xyz, ref hpr);
                    break;
                case 'f':
                    xyz.Z--;
                    ds.SetViewpoint(ref xyz, ref hpr);
                    break;
                case 'e':
                    xyz.Y++;
                    ds.SetViewpoint(ref xyz, ref hpr);
                    break;
                case 'q':
                    xyz.Y--;
                    ds.SetViewpoint(ref xyz, ref hpr);
                    break;
            }
        }

        public void step(int pause)
        {
            
            ds.SetColor(1.0f, 1.0f, 0.0f);
            ds.SetTexture(ds.Texture.Wood);
            lock (_prims)
            {
                foreach (OdePrim prm in _prims)
                {
                    //IntPtr body = d.GeomGetBody(prm.prim_geom);
                    if (prm.prim_geom != IntPtr.Zero)
                    {
                        d.Vector3 pos;
                        d.GeomCopyPosition(prm.prim_geom, out pos);
                        //d.BodyCopyPosition(body, out pos);

                        d.Matrix3 R;
                        d.GeomCopyRotation(prm.prim_geom, out R);
                        //d.BodyCopyRotation(body, out R);


                        d.Vector3 sides = new d.Vector3();
                        sides.X = prm.Size.X;
                        sides.Y = prm.Size.Y;
                        sides.Z = prm.Size.Z;

                        ds.DrawBox(ref pos, ref R, ref sides);
                    }
                }
            }
            ds.SetColor(1.0f, 0.0f, 0.0f);
            lock (_characters)
            {
                foreach (OdeCharacter chr in _characters)
                {
                    if (chr.Shell != IntPtr.Zero)
                    {
                        IntPtr body = d.GeomGetBody(chr.Shell);

                        d.Vector3 pos;
                        d.GeomCopyPosition(chr.Shell, out pos);
                        //d.BodyCopyPosition(body, out pos);

                        d.Matrix3 R;
                        d.GeomCopyRotation(chr.Shell, out R);
                        //d.BodyCopyRotation(body, out R);

                        ds.DrawCapsule(ref pos, ref R, chr.Size.Z, 0.35f);
                        d.Vector3 sides = new d.Vector3();
                        sides.X = 0.5f;
                        sides.Y = 0.5f;
                        sides.Z = 0.5f;

                        ds.DrawBox(ref pos, ref R, ref sides);
                    }
                }
            }
        }

        public void start(int unused)
        {
            ds.SetViewpoint(ref xyz, ref hpr);
        }
#endif
    }
}
