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
using System.Net;
using System.Xml;
using System.Reflection;
using System.Text;
using System.Threading;

using OpenSim.Framework;
using OpenSim.Framework.Capabilities;
using OpenSim.Framework.Client;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;
using OpenSim.Region.Framework.Scenes.Serialization;
using OpenSim.Services.Interfaces;

using GridRegion = OpenSim.Services.Interfaces.GridRegion;

using OpenMetaverse;
using log4net;
using Nini.Config;

namespace OpenSim.Region.CoreModules.Framework.InventoryAccess
{
    public class BasicInventoryAccessModule : INonSharedRegionModule, IInventoryAccessModule
    {
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        protected bool m_Enabled = false;
        protected Scene m_Scene;
        protected IUserManagement m_UserManagement;
        protected IUserManagement UserManagementModule
        {
            get
            {
                if (m_UserManagement == null)
                    m_UserManagement = m_Scene.RequestModuleInterface<IUserManagement>();
                return m_UserManagement;
            }
        }
        
        public bool CoalesceMultipleObjectsToInventory { get; set; }

        #region INonSharedRegionModule

        public Type ReplaceableInterface
        {
            get { return null; }
        }

        public virtual string Name
        {
            get { return "BasicInventoryAccessModule"; }
        }

        public virtual void Initialise(IConfigSource source)
        {
            IConfig moduleConfig = source.Configs["Modules"];
            if (moduleConfig != null)
            {
                string name = moduleConfig.GetString("InventoryAccessModule", "");
                if (name == Name)
                {
                    m_Enabled = true;
                    
                    InitialiseCommon(source);
                        
                    m_log.InfoFormat("[INVENTORY ACCESS MODULE]: {0} enabled.", Name);                                        
                }
            }
        }
        
        /// <summary>
        /// Common module config for both this and descendant classes.
        /// </summary>
        /// <param name="source"></param>
        protected virtual void InitialiseCommon(IConfigSource source)
        {
            IConfig inventoryConfig = source.Configs["Inventory"];
            
            if (inventoryConfig != null)
                CoalesceMultipleObjectsToInventory 
                    = inventoryConfig.GetBoolean("CoalesceMultipleObjectsToInventory", true);
            else
                CoalesceMultipleObjectsToInventory = true;
        }

        public virtual void PostInitialise()
        {
        }

        public virtual void AddRegion(Scene scene)
        {
            if (!m_Enabled)
                return;

            m_Scene = scene;

            scene.RegisterModuleInterface<IInventoryAccessModule>(this);
            scene.EventManager.OnNewClient += OnNewClient;
        }

        protected virtual void OnNewClient(IClientAPI client)
        {
            client.OnCreateNewInventoryItem += CreateNewInventoryItem;
        }

        public virtual void Close()
        {
            if (!m_Enabled)
                return;
        }


        public virtual void RemoveRegion(Scene scene)
        {
            if (!m_Enabled)
                return;
            m_Scene = null;
        }

        public virtual void RegionLoaded(Scene scene)
        {
            if (!m_Enabled)
                return;

        }

        #endregion

        #region Inventory Access

        /// <summary>
        /// Create a new inventory item.  Called when the client creates a new item directly within their
        /// inventory (e.g. by selecting a context inventory menu option).
        /// </summary>
        /// <param name="remoteClient"></param>
        /// <param name="transactionID"></param>
        /// <param name="folderID"></param>
        /// <param name="callbackID"></param>
        /// <param name="description"></param>
        /// <param name="name"></param>
        /// <param name="invType"></param>
        /// <param name="type"></param>
        /// <param name="wearableType"></param>
        /// <param name="nextOwnerMask"></param>
        public void CreateNewInventoryItem(IClientAPI remoteClient, UUID transactionID, UUID folderID,
                                           uint callbackID, string description, string name, sbyte invType,
                                           sbyte assetType,
                                           byte wearableType, uint nextOwnerMask, int creationDate)
        {
            m_log.DebugFormat("[AGENT INVENTORY]: Received request to create inventory item {0} in folder {1}", name, folderID);

            if (!m_Scene.Permissions.CanCreateUserInventory(invType, remoteClient.AgentId))
                return;

            InventoryFolderBase f = new InventoryFolderBase(folderID, remoteClient.AgentId);
            InventoryFolderBase folder = m_Scene.InventoryService.GetFolder(f);

            if (folder == null || folder.Owner != remoteClient.AgentId)
                return;

            if (transactionID == UUID.Zero)
            {
                ScenePresence presence;
                if (m_Scene.TryGetScenePresence(remoteClient.AgentId, out presence))
                {
                    byte[] data = null;

                    if (invType == (sbyte)InventoryType.Landmark && presence != null)
                    {
                        string suffix = string.Empty, prefix = string.Empty;
                        string strdata = GenerateLandmark(presence, out prefix, out suffix);
                        data = Encoding.ASCII.GetBytes(strdata);
                        name = prefix + name;
                        description += suffix;
                    }

                    AssetBase asset = m_Scene.CreateAsset(name, description, assetType, data, remoteClient.AgentId);
                    m_Scene.AssetService.Store(asset);

                    m_Scene.CreateNewInventoryItem(remoteClient, remoteClient.AgentId.ToString(), string.Empty, folderID, asset.Name, 0, callbackID, asset, invType, nextOwnerMask, creationDate);
                }
                else
                {
                    m_log.ErrorFormat(
                        "ScenePresence for agent uuid {0} unexpectedly not found in CreateNewInventoryItem",
                        remoteClient.AgentId);
                }
            }
            else
            {
                IAgentAssetTransactions agentTransactions = m_Scene.RequestModuleInterface<IAgentAssetTransactions>();
                if (agentTransactions != null)
                {
                    agentTransactions.HandleItemCreationFromTransaction(
                        remoteClient, transactionID, folderID, callbackID, description,
                        name, invType, assetType, wearableType, nextOwnerMask);
                }
            }
        }

        protected virtual string GenerateLandmark(ScenePresence presence, out string prefix, out string suffix)
        {
            prefix = string.Empty;
            suffix = string.Empty;
            Vector3 pos = presence.AbsolutePosition;
            return String.Format("Landmark version 2\nregion_id {0}\nlocal_pos {1} {2} {3}\nregion_handle {4}\n",
                                presence.Scene.RegionInfo.RegionID,
                                pos.X, pos.Y, pos.Z,
                                presence.RegionHandle);
        }

        /// <summary>
        /// Capability originating call to update the asset of an item in an agent's inventory
        /// </summary>
        /// <param name="remoteClient"></param>
        /// <param name="itemID"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public virtual UUID CapsUpdateInventoryItemAsset(IClientAPI remoteClient, UUID itemID, byte[] data)
        {
            InventoryItemBase item = new InventoryItemBase(itemID, remoteClient.AgentId);
            item = m_Scene.InventoryService.GetItem(item);

            if (item.Owner != remoteClient.AgentId)
                return UUID.Zero;

            if (item != null)
            {
                if ((InventoryType)item.InvType == InventoryType.Notecard)
                {
                    if (!m_Scene.Permissions.CanEditNotecard(itemID, UUID.Zero, remoteClient.AgentId))
                    {
                        remoteClient.SendAgentAlertMessage("Insufficient permissions to edit notecard", false);
                        return UUID.Zero;
                    }

                    remoteClient.SendAgentAlertMessage("Notecard saved", false);
                }
                else if ((InventoryType)item.InvType == InventoryType.LSL)
                {
                    if (!m_Scene.Permissions.CanEditScript(itemID, UUID.Zero, remoteClient.AgentId))
                    {
                        remoteClient.SendAgentAlertMessage("Insufficient permissions to edit script", false);
                        return UUID.Zero;
                    }

                    remoteClient.SendAgentAlertMessage("Script saved", false);
                }

                AssetBase asset =
                    CreateAsset(item.Name, item.Description, (sbyte)item.AssetType, data, remoteClient.AgentId.ToString());
                item.AssetID = asset.FullID;
                m_Scene.AssetService.Store(asset);

                m_Scene.InventoryService.UpdateItem(item);

                // remoteClient.SendInventoryItemCreateUpdate(item);
                return (asset.FullID);
            }
            else
            {
                m_log.ErrorFormat(
                    "[AGENT INVENTORY]: Could not find item {0} for caps inventory update",
                    itemID);
            }

            return UUID.Zero;
        }
        
        public virtual UUID CopyToInventory(DeRezAction action, UUID folderID,
                List<SceneObjectGroup> objectGroups, IClientAPI remoteClient)
        {
            Dictionary<UUID, List<SceneObjectGroup>> bundlesToCopy = new Dictionary<UUID, List<SceneObjectGroup>>();
            
            if (CoalesceMultipleObjectsToInventory)
            {
                // The following code groups the SOG's by owner. No objects
                // belonging to different people can be coalesced, for obvious
                // reasons.
                foreach (SceneObjectGroup g in objectGroups)
                {
                    if (!bundlesToCopy.ContainsKey(g.OwnerID))
                        bundlesToCopy[g.OwnerID] = new List<SceneObjectGroup>();
    
                    bundlesToCopy[g.OwnerID].Add(g);
                }
            }
            else
            {
                // If we don't want to coalesce then put every object in its own bundle.
                foreach (SceneObjectGroup g in objectGroups)
                {
                    List<SceneObjectGroup> bundle = new List<SceneObjectGroup>();
                    bundle.Add(g);
                    bundlesToCopy[g.UUID] = bundle;                    
                }
            }

            // This is method scoped and will be returned. It will be the
            // last created asset id
            UUID assetID = UUID.Zero;

            // Each iteration is really a separate asset being created,
            // with distinct destinations as well.
            foreach (List<SceneObjectGroup> bundle in bundlesToCopy.Values)
                assetID = CopyBundleToInventory(action, folderID, bundle, remoteClient);
            
            return assetID;
        }
        
        /// <summary>
        /// Copy a bundle of objects to inventory.  If there is only one object, then this will create an object
        /// item.  If there are multiple objects then these will be saved as a single coalesced item.
        /// </summary>
        /// <param name="action"></param>
        /// <param name="folderID"></param>
        /// <param name="objlist"></param>
        /// <param name="remoteClient"></param>
        /// <returns></returns>
        protected UUID CopyBundleToInventory(
            DeRezAction action, UUID folderID, List<SceneObjectGroup> objlist, IClientAPI remoteClient)
        {
            UUID assetID = UUID.Zero;
            
            CoalescedSceneObjects coa = new CoalescedSceneObjects(UUID.Zero);                
            Dictionary<UUID, Vector3> originalPositions = new Dictionary<UUID, Vector3>();

            foreach (SceneObjectGroup objectGroup in objlist)
            {
                Vector3 inventoryStoredPosition = new Vector3
                            (((objectGroup.AbsolutePosition.X > (int)Constants.RegionSize)
                                  ? 250
                                  : objectGroup.AbsolutePosition.X)
                             ,
                             (objectGroup.AbsolutePosition.Y > (int)Constants.RegionSize)
                                 ? 250
                                 : objectGroup.AbsolutePosition.Y,
                             objectGroup.AbsolutePosition.Z);

                originalPositions[objectGroup.UUID] = objectGroup.AbsolutePosition;

                objectGroup.AbsolutePosition = inventoryStoredPosition;

                // Make sure all bits but the ones we want are clear
                // on take.
                // This will be applied to the current perms, so
                // it will do what we want.
                objectGroup.RootPart.NextOwnerMask &=
                        ((uint)PermissionMask.Copy |
                         (uint)PermissionMask.Transfer |
                         (uint)PermissionMask.Modify);
                objectGroup.RootPart.NextOwnerMask |=
                        (uint)PermissionMask.Move;
                
                coa.Add(objectGroup);
            }

            string itemXml;

            if (objlist.Count > 1)
                itemXml = CoalescedSceneObjectsSerializer.ToXml(coa);
            else
                itemXml = SceneObjectSerializer.ToOriginalXmlFormat(objlist[0]);
            
            // Restore the position of each group now that it has been stored to inventory.
            foreach (SceneObjectGroup objectGroup in objlist)
                objectGroup.AbsolutePosition = originalPositions[objectGroup.UUID];

            InventoryItemBase item = CreateItemForObject(action, remoteClient, objlist[0], folderID);
            if (item == null)
                return UUID.Zero;
                            
            // Can't know creator is the same, so null it in inventory
            if (objlist.Count > 1)
            {
                item.CreatorId = UUID.Zero.ToString();
                item.Flags = (uint)InventoryItemFlags.ObjectHasMultipleItems;
            }
            else
            {
                item.CreatorId = objlist[0].RootPart.CreatorID.ToString();                
                item.SaleType = objlist[0].RootPart.ObjectSaleType;
                item.SalePrice = objlist[0].RootPart.SalePrice;                    
            }              

            AssetBase asset = CreateAsset(
                objlist[0].GetPartName(objlist[0].RootPart.LocalId),
                objlist[0].GetPartDescription(objlist[0].RootPart.LocalId),
                (sbyte)AssetType.Object,
                Utils.StringToBytes(itemXml),
                objlist[0].OwnerID.ToString());
            m_Scene.AssetService.Store(asset);
            
            item.AssetID = asset.FullID;  
            assetID = asset.FullID;                              

            if (DeRezAction.SaveToExistingUserInventoryItem == action)
            {
                m_Scene.InventoryService.UpdateItem(item);
            }
            else
            {
                AddPermissions(item, objlist[0], objlist, remoteClient);

                item.CreationDate = Util.UnixTimeSinceEpoch();
                item.Description = asset.Description;
                item.Name = asset.Name;
                item.AssetType = asset.Type;

                m_Scene.AddInventoryItem(item);

                if (remoteClient != null && item.Owner == remoteClient.AgentId)
                {
                    remoteClient.SendInventoryItemCreateUpdate(item, 0);
                }
                else
                {
                    ScenePresence notifyUser = m_Scene.GetScenePresence(item.Owner);
                    if (notifyUser != null)
                    {
                        notifyUser.ControllingClient.SendInventoryItemCreateUpdate(item, 0);
                    }
                }
            }

            // This is a hook to do some per-asset post-processing for subclasses that need that
            if (remoteClient != null)
                ExportAsset(remoteClient.AgentId, assetID);
            
            return assetID;
        }

        protected virtual void ExportAsset(UUID agentID, UUID assetID)
        {
            // nothing to do here
        }

        /// <summary>
        /// Add relevant permissions for an object to the item.
        /// </summary>
        /// <param name="item"></param>
        /// <param name="so"></param>
        /// <param name="objsForEffectivePermissions"></param>
        /// <param name="remoteClient"></param>
        /// <returns></returns>
        protected InventoryItemBase AddPermissions(
            InventoryItemBase item, SceneObjectGroup so, List<SceneObjectGroup> objsForEffectivePermissions, 
            IClientAPI remoteClient)
        {
            uint effectivePerms = (uint)(PermissionMask.Copy | PermissionMask.Transfer | PermissionMask.Modify | PermissionMask.Move) | 7;
            foreach (SceneObjectGroup grp in objsForEffectivePermissions)
                effectivePerms &= grp.GetEffectivePermissions();
            effectivePerms |= (uint)PermissionMask.Move;

            if (remoteClient != null && (remoteClient.AgentId != so.RootPart.OwnerID) && m_Scene.Permissions.PropagatePermissions())
            {
                uint perms = effectivePerms;
                uint nextPerms = (perms & 7) << 13;
                if ((nextPerms & (uint)PermissionMask.Copy) == 0)
                    perms &= ~(uint)PermissionMask.Copy;
                if ((nextPerms & (uint)PermissionMask.Transfer) == 0)
                    perms &= ~(uint)PermissionMask.Transfer;
                if ((nextPerms & (uint)PermissionMask.Modify) == 0)
                    perms &= ~(uint)PermissionMask.Modify;

                item.BasePermissions = perms & so.RootPart.NextOwnerMask;
                item.CurrentPermissions = item.BasePermissions;
                item.NextPermissions = perms & so.RootPart.NextOwnerMask;
                item.EveryOnePermissions = so.RootPart.EveryoneMask & so.RootPart.NextOwnerMask;
                item.GroupPermissions = so.RootPart.GroupMask & so.RootPart.NextOwnerMask;
                
                // Magic number badness. Maybe this deserves an enum.
                // bit 4 (16) is the "Slam" bit, it means treat as passed
                // and apply next owner perms on rez
                item.CurrentPermissions |= 16; // Slam!
            }
            else
            {
                item.BasePermissions = effectivePerms;
                item.CurrentPermissions = effectivePerms;
                item.NextPermissions = so.RootPart.NextOwnerMask & effectivePerms;
                item.EveryOnePermissions = so.RootPart.EveryoneMask & effectivePerms;
                item.GroupPermissions = so.RootPart.GroupMask & effectivePerms;

                item.CurrentPermissions &=
                        ((uint)PermissionMask.Copy |
                         (uint)PermissionMask.Transfer |
                         (uint)PermissionMask.Modify |
                         (uint)PermissionMask.Move |
                         7); // Preserve folded permissions
            }    
            
            return item;
        }
        
        /// <summary>
        /// Create an item using details for the given scene object.
        /// </summary>
        /// <param name="action"></param>
        /// <param name="remoteClient"></param>
        /// <param name="so"></param>
        /// <param name="folderID"></param>
        /// <returns></returns>
        protected InventoryItemBase CreateItemForObject(
            DeRezAction action, IClientAPI remoteClient, SceneObjectGroup so, UUID folderID)
        {
            // Get the user info of the item destination
            //
            UUID userID = UUID.Zero;

            if (action == DeRezAction.Take || action == DeRezAction.TakeCopy ||
                action == DeRezAction.SaveToExistingUserInventoryItem)
            {
                // Take or take copy require a taker
                // Saving changes requires a local user
                //
                if (remoteClient == null)
                    return null;

                userID = remoteClient.AgentId;
            }
            else
            {
                // All returns / deletes go to the object owner
                //
                userID = so.RootPart.OwnerID;
            }

            if (userID == UUID.Zero) // Can't proceed
            {
                return null;
            }

            // If we're returning someone's item, it goes back to the
            // owner's Lost And Found folder.
            // Delete is treated like return in this case
            // Deleting your own items makes them go to trash
            //
            
            InventoryFolderBase folder = null;
            InventoryItemBase item = null;

            if (DeRezAction.SaveToExistingUserInventoryItem == action)
            {
                item = new InventoryItemBase(so.RootPart.FromUserInventoryItemID, userID);
                item = m_Scene.InventoryService.GetItem(item);

                //item = userInfo.RootFolder.FindItem(
                //        objectGroup.RootPart.FromUserInventoryItemID);

                if (null == item)
                {
                    m_log.DebugFormat(
                        "[AGENT INVENTORY]: Object {0} {1} scheduled for save to inventory has already been deleted.",
                        so.Name, so.UUID);
                    
                    return null;
                }
            }
            else
            {
                // Folder magic
                //
                if (action == DeRezAction.Delete)
                {
                    // Deleting someone else's item
                    //
                    if (remoteClient == null ||
                        so.OwnerID != remoteClient.AgentId)
                    {
                        folder = m_Scene.InventoryService.GetFolderForType(userID, AssetType.LostAndFoundFolder);
                    }
                    else
                    {
                        folder = m_Scene.InventoryService.GetFolderForType(userID, AssetType.TrashFolder);
                    }
                }
                else if (action == DeRezAction.Return)
                {
                    // Dump to lost + found unconditionally
                    //
                    folder = m_Scene.InventoryService.GetFolderForType(userID, AssetType.LostAndFoundFolder);
                }

                if (folderID == UUID.Zero && folder == null)
                {
                    if (action == DeRezAction.Delete)
                    {
                        // Deletes go to trash by default
                        //
                        folder = m_Scene.InventoryService.GetFolderForType(userID, AssetType.TrashFolder);
                    }
                    else
                    {
                        if (remoteClient == null || so.OwnerID != remoteClient.AgentId)
                        {
                            // Taking copy of another person's item. Take to
                            // Objects folder.
                            folder = m_Scene.InventoryService.GetFolderForType(userID, AssetType.Object);
                        }
                        else
                        {
                            // Catch all. Use lost & found
                            //

                            folder = m_Scene.InventoryService.GetFolderForType(userID, AssetType.LostAndFoundFolder);
                        }
                    }
                }

                // Override and put into where it came from, if it came
                // from anywhere in inventory
                //
                if (action == DeRezAction.Take || action == DeRezAction.TakeCopy)
                {
                    if (so.RootPart.FromFolderID != UUID.Zero)
                    {
                        InventoryFolderBase f = new InventoryFolderBase(so.RootPart.FromFolderID, userID);
                        folder = m_Scene.InventoryService.GetFolder(f);
                    }
                }

                if (folder == null) // None of the above
                {
                    folder = new InventoryFolderBase(folderID);

                    if (folder == null) // Nowhere to put it
                    {
                        return null;
                    }
                }

                item = new InventoryItemBase();                
                item.ID = UUID.Random();
                item.InvType = (int)InventoryType.Object;
                item.Folder = folder.ID;
                item.Owner = userID;
            }   
            
            return item;
        }

        public virtual SceneObjectGroup RezObject(
            IClientAPI remoteClient, UUID itemID, Vector3 RayEnd, Vector3 RayStart,
            UUID RayTargetID, byte BypassRayCast, bool RayEndIsIntersection,
            bool RezSelected, bool RemoveItem, UUID fromTaskID, bool attachment)
        {
//            m_log.DebugFormat("[INVENTORY ACCESS MODULE]: RezObject for {0}, item {1}", remoteClient.Name, itemID);

            InventoryItemBase item = new InventoryItemBase(itemID, remoteClient.AgentId);
            item = m_Scene.InventoryService.GetItem(item);

            if (item == null)
            {
                m_log.WarnFormat(
                    "[InventoryAccessModule]: Could not find item {0} for {1} in RezObject()",
                    itemID, remoteClient.Name);

                return null;
            }

            item.Owner = remoteClient.AgentId;

            return RezObject(
                remoteClient, item, item.AssetID,
                RayEnd, RayStart, RayTargetID, BypassRayCast, RayEndIsIntersection,
                RezSelected, RemoveItem, fromTaskID, attachment);
        }

        public virtual SceneObjectGroup RezObject(
            IClientAPI remoteClient, InventoryItemBase item, UUID assetID, Vector3 RayEnd, Vector3 RayStart,
            UUID RayTargetID, byte BypassRayCast, bool RayEndIsIntersection,
            bool RezSelected, bool RemoveItem, UUID fromTaskID, bool attachment)
        {
            AssetBase rezAsset = m_Scene.AssetService.Get(assetID.ToString());

            if (rezAsset == null)
            {
                if (item != null)
                {
                    m_log.WarnFormat(
                        "[InventoryAccessModule]: Could not find asset {0} for item {1} {2} for {3} in RezObject()",
                        assetID, item.Name, item.ID, remoteClient.Name);
                }
                else
                {
                    m_log.WarnFormat(
                        "[InventoryAccessModule]: Could not find asset {0} for {1} in RezObject()",
                        assetID, remoteClient.Name);
                }

                return null;
            }

            SceneObjectGroup group = null;

            string xmlData = Utils.BytesToString(rezAsset.Data);
            List<SceneObjectGroup> objlist = new List<SceneObjectGroup>();
            List<Vector3> veclist = new List<Vector3>();
            byte bRayEndIsIntersection = (byte)(RayEndIsIntersection ? 1 : 0);
            Vector3 pos;

            XmlDocument doc = new XmlDocument();
            doc.LoadXml(xmlData);
            XmlElement e = (XmlElement)doc.SelectSingleNode("/CoalescedObject");
            if (e == null || attachment) // Single
            {
                SceneObjectGroup g = SceneObjectSerializer.FromOriginalXmlFormat(xmlData);

                objlist.Add(g);
                veclist.Add(new Vector3(0, 0, 0));

                float offsetHeight = 0;
                pos = m_Scene.GetNewRezLocation(
                    RayStart, RayEnd, RayTargetID, Quaternion.Identity,
                    BypassRayCast, bRayEndIsIntersection, true, g.GetAxisAlignedBoundingBox(out offsetHeight), false);
                pos.Z += offsetHeight;
            }
            else
            {
                XmlElement coll = (XmlElement)e;
                float bx = Convert.ToSingle(coll.GetAttribute("x"));
                float by = Convert.ToSingle(coll.GetAttribute("y"));
                float bz = Convert.ToSingle(coll.GetAttribute("z"));
                Vector3 bbox = new Vector3(bx, by, bz);

                pos = m_Scene.GetNewRezLocation(RayStart, RayEnd,
                        RayTargetID, Quaternion.Identity,
                        BypassRayCast, bRayEndIsIntersection, true,
                        bbox, false);

                pos -= bbox / 2;

                XmlNodeList groups = e.SelectNodes("SceneObjectGroup");
                foreach (XmlNode n in groups)
                {
                    SceneObjectGroup g = SceneObjectSerializer.FromOriginalXmlFormat(n.OuterXml);

                    objlist.Add(g);
                    XmlElement el = (XmlElement)n;

                    string rawX = el.GetAttribute("offsetx");
                    string rawY = el.GetAttribute("offsety");
                    string rawZ = el.GetAttribute("offsetz");
//                        
//                            m_log.DebugFormat(
//                                "[INVENTORY ACCESS MODULE]: Converting coalesced object {0} offset <{1}, {2}, {3}>", 
//                                g.Name, rawX, rawY, rawZ);

                    float x = Convert.ToSingle(rawX);
                    float y = Convert.ToSingle(rawY);
                    float z = Convert.ToSingle(rawZ);
                    veclist.Add(new Vector3(x, y, z));
                }
            }

            if (item != null && !DoPreRezWhenFromItem(remoteClient, item, objlist, pos, attachment))
                return null;

            for (int i = 0; i < objlist.Count; i++)
            {
                group = objlist[i];

//                        Vector3 storedPosition = group.AbsolutePosition;
                if (group.UUID == UUID.Zero)
                {
                    m_log.Debug("[InventoryAccessModule]: Object has UUID.Zero! Position 3");
                }

                foreach (SceneObjectPart part in group.Parts)
                {
                    // Make the rezzer the owner, as this is not necessarily set correctly in the serialized asset.
                    part.LastOwnerID = part.OwnerID;
                    part.OwnerID = remoteClient.AgentId;
                }

                if (!attachment)
                {
                    // If it's rezzed in world, select it. Much easier to
                    // find small items.
                    //
                    foreach (SceneObjectPart part in group.Parts)
                    {
                        part.CreateSelected = true;
                    }
                }

                group.ResetIDs();

                if (attachment)
                {
                    group.RootPart.Flags |= PrimFlags.Phantom;
                    group.IsAttachment = true;
                }

                // If we're rezzing an attachment then don't ask
                // AddNewSceneObject() to update the client since
                // we'll be doing that later on.  Scheduling more than
                // one full update during the attachment
                // process causes some clients to fail to display the
                // attachment properly.
                m_Scene.AddNewSceneObject(group, true, false);

                // if attachment we set it's asset id so object updates
                // can reflect that, if not, we set it's position in world.
                if (!attachment)
                {
                    group.ScheduleGroupForFullUpdate();

                    group.AbsolutePosition = pos + veclist[i];
                }

                group.SetGroup(remoteClient.ActiveGroupId, remoteClient);

                if (!attachment)
                {
                    SceneObjectPart rootPart = group.RootPart;

                    if (rootPart.Shape.PCode == (byte)PCode.Prim)
                        group.ClearPartAttachmentData();

                    // Fire on_rez
                    group.CreateScriptInstances(0, true, m_Scene.DefaultScriptEngine, 1);
                    rootPart.ParentGroup.ResumeScripts();

                    rootPart.ScheduleFullUpdate();
                }

//                        m_log.DebugFormat(
//                            "[InventoryAccessModule]: Rezzed {0} {1} {2} for {3}",
//                            group.Name, group.LocalId, group.UUID, remoteClient.Name);
            }

            if (item != null)
                DoPostRezWhenFromItem(item, attachment);

            return group;
        }

        /// <summary>
        /// Do pre-rez processing when the object comes from an item.
        /// </summary>
        /// <param name="remoteClient"></param>
        /// <param name="item"></param>
        /// <param name="objlist"></param>
        /// <param name="pos"></param>
        /// <param name="isAttachment"></param>
        /// <returns>true if we can processed with rezzing, false if we need to abort</returns>
        private bool DoPreRezWhenFromItem(
            IClientAPI remoteClient, InventoryItemBase item, List<SceneObjectGroup> objlist, Vector3 pos, bool isAttachment)
        {
            UUID fromUserInventoryItemId = UUID.Zero;

            // If we have permission to copy then link the rezzed object back to the user inventory
            // item that it came from.  This allows us to enable 'save object to inventory'
            if (!m_Scene.Permissions.BypassPermissions())
            {
                if ((item.CurrentPermissions & (uint)PermissionMask.Copy)
                    == (uint)PermissionMask.Copy && (item.Flags & (uint)InventoryItemFlags.ObjectHasMultipleItems) == 0)
                {
                    fromUserInventoryItemId = item.ID;
                }
            }
            else
            {
                if ((item.Flags & (uint)InventoryItemFlags.ObjectHasMultipleItems) == 0)
                {
                    // Brave new fullperm world
                    fromUserInventoryItemId = item.ID;
                }
            }

            int primcount = 0;
            foreach (SceneObjectGroup g in objlist)
                primcount += g.PrimCount;

            if (!m_Scene.Permissions.CanRezObject(
                primcount, remoteClient.AgentId, pos)
                && !isAttachment)
            {
                // The client operates in no fail mode. It will
                // have already removed the item from the folder
                // if it's no copy.
                // Put it back if it's not an attachment
                //
                if (((item.CurrentPermissions & (uint)PermissionMask.Copy) == 0) && (!isAttachment))
                    remoteClient.SendBulkUpdateInventory(item);

                return false;
            }

            for (int i = 0; i < objlist.Count; i++)
            {
                SceneObjectGroup so = objlist[i];
                SceneObjectPart rootPart = so.RootPart;

                // Since renaming the item in the inventory does not
                // affect the name stored in the serialization, transfer
                // the correct name from the inventory to the
                // object itself before we rez.
                //
                // Only do these for the first object if we are rezzing a coalescence.
                if (i == 0)
                {
                    rootPart.Name = item.Name;
                    rootPart.Description = item.Description;
                    rootPart.ObjectSaleType = item.SaleType;
                    rootPart.SalePrice = item.SalePrice;
                }

                rootPart.FromFolderID = item.Folder;
    
                if ((rootPart.OwnerID != item.Owner) ||
                    (item.CurrentPermissions & 16) != 0)
                {
                    //Need to kill the for sale here
                    rootPart.ObjectSaleType = 0;
                    rootPart.SalePrice = 10;
    
                    if (m_Scene.Permissions.PropagatePermissions())
                    {
                        foreach (SceneObjectPart part in so.Parts)
                        {
                            if ((item.Flags & (uint)InventoryItemFlags.ObjectHasMultipleItems) == 0)
                            {
                                part.EveryoneMask = item.EveryOnePermissions;
                                part.NextOwnerMask = item.NextPermissions;
                            }
                            part.GroupMask = 0; // DO NOT propagate here
                        }
    
                        so.ApplyNextOwnerPermissions();
                    }
                }
    
                foreach (SceneObjectPart part in so.Parts)
                {
                    part.FromUserInventoryItemID = fromUserInventoryItemId;

                    if ((part.OwnerID != item.Owner) ||
                        (item.CurrentPermissions & 16) != 0)
                    {
                        part.Inventory.ChangeInventoryOwner(item.Owner);
                        part.GroupMask = 0; // DO NOT propagate here
                    }

                    part.EveryoneMask = item.EveryOnePermissions;
                    part.NextOwnerMask = item.NextPermissions;
                }
    
                rootPart.TrimPermissions();

                if (isAttachment)
                    so.SetFromItemID(item.ID);
            }

            return true;
        }

        /// <summary>
        /// Do post-rez processing when the object comes from an item.
        /// </summary>
        /// <param name="item"></param>
        /// <param name="isAttachment"></param>
        private void DoPostRezWhenFromItem(InventoryItemBase item, bool isAttachment)
        {
            if (!m_Scene.Permissions.BypassPermissions())
            {
                if ((item.CurrentPermissions & (uint)PermissionMask.Copy) == 0)
                {
                    // If this is done on attachments, no
                    // copy ones will be lost, so avoid it
                    //
                    if (!isAttachment)
                    {
                        List<UUID> uuids = new List<UUID>();
                        uuids.Add(item.ID);
                        m_Scene.InventoryService.DeleteItems(item.Owner, uuids);
                    }
                }
            }
        }

        protected void AddUserData(SceneObjectGroup sog)
        {
            UserManagementModule.AddUser(sog.RootPart.CreatorID, sog.RootPart.CreatorData);
            foreach (SceneObjectPart sop in sog.Parts)
                UserManagementModule.AddUser(sop.CreatorID, sop.CreatorData);
        }

        public virtual void TransferInventoryAssets(InventoryItemBase item, UUID sender, UUID receiver)
        {
        }

        public virtual bool CanGetAgentInventoryItem(IClientAPI remoteClient, UUID itemID, UUID requestID)
        {
            InventoryItemBase assetRequestItem = GetItem(remoteClient.AgentId, itemID);

            if (assetRequestItem == null)
            {
                ILibraryService lib = m_Scene.RequestModuleInterface<ILibraryService>();

                if (lib != null)
                    assetRequestItem = lib.LibraryRootFolder.FindItem(itemID);

                if (assetRequestItem == null)
                    return false;
            }

            // At this point, we need to apply perms
            // only to notecards and scripts. All
            // other asset types are always available
            //
            if (assetRequestItem.AssetType == (int)AssetType.LSLText)
            {
                if (!m_Scene.Permissions.CanViewScript(itemID, UUID.Zero, remoteClient.AgentId))
                {
                    remoteClient.SendAgentAlertMessage("Insufficient permissions to view script", false);
                    return false;
                }
            }
            else if (assetRequestItem.AssetType == (int)AssetType.Notecard)
            {
                if (!m_Scene.Permissions.CanViewNotecard(itemID, UUID.Zero, remoteClient.AgentId))
                {
                    remoteClient.SendAgentAlertMessage("Insufficient permissions to view notecard", false);
                    return false;
                }
            }

            if (assetRequestItem.AssetID != requestID)
            {
                m_log.WarnFormat(
                    "[CLIENT]: {0} requested asset {1} from item {2} but this does not match item's asset {3}",
                    Name, requestID, itemID, assetRequestItem.AssetID);

                return false;
            }

            return true;
        }


        public virtual bool IsForeignUser(UUID userID, out string assetServerURL)
        {
            assetServerURL = string.Empty;
            return false;
        }

        #endregion

        #region Misc

        /// <summary>
        /// Create a new asset data structure.
        /// </summary>
        /// <param name="name"></param>
        /// <param name="description"></param>
        /// <param name="invType"></param>
        /// <param name="assetType"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        private AssetBase CreateAsset(string name, string description, sbyte assetType, byte[] data, string creatorID)
        {
            AssetBase asset = new AssetBase(UUID.Random(), name, assetType, creatorID);
            asset.Description = description;
            asset.Data = (data == null) ? new byte[1] : data;

            return asset;
        }

        protected virtual InventoryItemBase GetItem(UUID agentID, UUID itemID)
        {
            IInventoryService invService = m_Scene.RequestModuleInterface<IInventoryService>();
            InventoryItemBase item = new InventoryItemBase(itemID, agentID);
            item = invService.GetItem(item);
            
            if (item != null && item.CreatorData != null && item.CreatorData != string.Empty)
                UserManagementModule.AddUser(item.CreatorIdAsUuid, item.CreatorData);

            return item;
        }

        #endregion
    }
}