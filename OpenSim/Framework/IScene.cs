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

using OpenMetaverse;
//using OpenSim.Framework.Console;
using Nini.Config;

namespace OpenSim.Framework
{
    public delegate void restart(RegionInfo thisRegion);

    public enum RegionStatus : int
    {
        Down = 0,
        Up = 1,
        Crashed = 2,
        Starting = 3,
    };
            
    /// <value>
    /// Indicate what action to take on an object derez request
    /// </value>
    public enum DeRezAction : byte
    {
        SaveToExistingUserInventoryItem = 0,
        TakeCopy = 1,
        Take = 4,
        GodTakeCopy = 5,
        Delete = 6,
        Return = 9
    };

    public interface IScene
    {
        RegionInfo RegionInfo { get; }
        RegionStatus RegionStatus { get; set; }

        IConfigSource Config { get; }

        float TimeDilation { get; }

        bool AllowScriptCrossings { get; }

        event restart OnRestart;

        /// <summary>
        /// Add a new client and create a presence for it.  All clients except initial login clients will starts off as a child agent
        /// - the later agent crossing will promote it to a root agent.
        /// </summary>
        /// <param name="client"></param>
        /// <param name="type">The type of agent to add.</param>
        /// <returns>
        /// The scene agent if the new client was added or if an agent that already existed.</returns>
        ISceneAgent AddNewClient(IClientAPI client, PresenceType type);

        /// <summary>
        /// Remove the given client from the scene.
        /// </summary>
        /// <param name="agentID"></param>
        /// <param name="closeChildAgents">Close the neighbour child agents associated with this client.</param>
        void RemoveClient(UUID agentID, bool closeChildAgents);

        void Restart();
        //RegionInfo OtherRegionUp(RegionInfo thisRegion);

        string GetSimulatorVersion();

        /// <summary>
        /// Is the agent denoted by the given agentID a child presence in this scene?
        /// </summary>
        /// <remarks>
        /// Used by ClientView when a 'kick everyone' or 'estate message' occurs
        /// </remarks>
        /// <param name="avatarID">AvatarID to lookup</param>
        /// <returns>true if the presence is a child agent, false if the presence is a root exception</returns>
        /// <exception cref="System.NullReferenceException">
        /// Thrown if the agent does not exist.
        /// </exception>
        bool PresenceChildStatus(UUID agentId);

        bool TryGetScenePresence(UUID agentID, out object scenePresence);

        /// <summary>
        /// Register an interface to a region module.  This allows module methods to be called directly as
        /// well as via events.  If there is already a module registered for this interface, it is not replaced
        /// (is this the best behaviour?)
        /// </summary>
        /// <param name="mod"></param>
        void RegisterModuleInterface<M>(M mod);
        
        void StackModuleInterface<M>(M mod);

        /// <summary>
        /// For the given interface, retrieve the region module which implements it.
        /// </summary>
        /// <returns>null if there is no registered module implementing that interface</returns>
        T RequestModuleInterface<T>();

        /// <summary>
        /// For the given interface, retrieve an array of region modules that implement it.
        /// </summary>
        /// <returns>an empty array if there are no registered modules implementing that interface</returns>
        T[] RequestModuleInterfaces<T>();

//        void AddCommand(object module, string command, string shorthelp, string longhelp, CommandDelegate callback);

        ISceneObject DeserializeObject(string representation);

        bool CheckClient(UUID agentID, System.Net.IPEndPoint ep);
    }
}
