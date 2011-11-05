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
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.CoreModules.Avatar.Inventory.Archiver;
using OpenSim.Region.Framework.Scenes;
using OpenSim.Services.Interfaces;

namespace OpenSim.Tests.Common
{
    /// <summary>
    /// Utility functions for carrying out user inventory tests.
    /// </summary>
    public static class UserInventoryHelpers
    {
        public static readonly string PATH_DELIMITER = "/";

        /// <summary>
        /// Creates a notecard in the objects folder and specify an item id.
        /// </summary>
        /// <param name="scene"></param>
        /// <param name="itemName"></param>
        /// <param name="itemId"></param>
        /// <param name="userId"></param>
        /// <returns></returns>
        public static InventoryItemBase CreateInventoryItem(Scene scene, string itemName, UUID userId)
        {
            return CreateInventoryItem(scene, itemName, UUID.Random(), UUID.Random(), userId, InventoryType.Notecard);
        }

        /// <summary>
        /// Creates an item of the given type with an accompanying asset.
        /// </summary>
        /// <param name="scene"></param>
        /// <param name="itemName"></param>
        /// <param name="itemId"></param>
        /// <param name="userId"></param>
        /// <param name="type">Type of item to create</param>
        /// <returns></returns>
        public static InventoryItemBase CreateInventoryItem(
            Scene scene, string itemName, UUID userId, InventoryType type)
        {
            return CreateInventoryItem(scene, itemName, UUID.Random(), UUID.Random(), userId, type);
        }

        /// <summary>
        /// Creates a notecard in the objects folder and specify an item id.
        /// </summary>
        /// <param name="scene"></param>
        /// <param name="itemName"></param>
        /// <param name="itemId"></param>
        /// <param name="assetId"></param>
        /// <param name="userId"></param>
        /// <param name="type">Type of item to create</param>
        /// <returns></returns>
        public static InventoryItemBase CreateInventoryItem(
            Scene scene, string itemName, UUID itemId, UUID assetId, UUID userId, InventoryType type)
        {
            AssetBase asset = null;

            if (type == InventoryType.Notecard)
            {
                asset = AssetHelpers.CreateNotecardAsset();
                asset.CreatorID = userId.ToString();
            }
            else if (type == InventoryType.Object)
            {
                asset = AssetHelpers.CreateAsset(assetId, SceneHelpers.CreateSceneObject(1, userId));
            }
            else
            {
                throw new Exception(string.Format("Inventory type {0} not supported", type));
            }

            scene.AssetService.Store(asset);

            InventoryItemBase item = new InventoryItemBase();
            item.Name = itemName;
            item.AssetID = asset.FullID;
            item.ID = itemId;
            item.Owner = userId;
            item.AssetType = asset.Type;
            item.InvType = (int)type;

            InventoryFolderBase folder = scene.InventoryService.GetFolderForType(userId, AssetType.Notecard);
            
            item.Folder = folder.ID;
            scene.AddInventoryItem(item);
            
            return item;
        }
        
        /// <summary>
        /// Create inventory folders starting from the user's root folder.
        /// </summary>
        ///
        /// Ignores any existing folders with the same name
        /// 
        /// <param name="inventoryService"></param>
        /// <param name="userId"></param>
        /// <param name="path">
        /// The folders to create.  Multiple folders can be specified on a path delimited by the PATH_DELIMITER
        /// </param>
        /// <returns>
        /// The folder created.  If the path contains multiple folders then the last one created is returned.
        /// Will return null if the root folder could not be found.
        /// </returns>
        public static InventoryFolderBase CreateInventoryFolder(
            IInventoryService inventoryService, UUID userId, string path)
        {
            InventoryFolderBase rootFolder = inventoryService.GetRootFolder(userId);

            if (null == rootFolder)
                return null;

            return CreateInventoryFolder(inventoryService, rootFolder, path);
        }

        /// <summary>
        /// Create inventory folders starting from a given parent folder
        /// </summary>
        ///
        /// Ignores any existing folders with the same name
        /// 
        /// <param name="inventoryService"></param>
        /// <param name="parentFolder"></param>
        /// <param name="path">
        /// The folders to create.  Multiple folders can be specified on a path delimited by the PATH_DELIMITER
        /// </param>
        /// <returns>
        /// The folder created.  If the path contains multiple folders then the last one created is returned.
        /// </returns>
        public static InventoryFolderBase CreateInventoryFolder(
            IInventoryService inventoryService, InventoryFolderBase parentFolder, string path)
        {
            string[] components = path.Split(new string[] { PATH_DELIMITER }, 2, StringSplitOptions.None);

            InventoryFolderBase newFolder 
                = new InventoryFolderBase(UUID.Random(), components[0], parentFolder.Owner, parentFolder.ID);
            inventoryService.AddFolder(newFolder);

            if (components.Length > 1)
                return CreateInventoryFolder(inventoryService, newFolder, components[1]);
            else
                return newFolder;
        }

        /// <summary>
        /// Get the inventory folder that matches the path name.  If there are multiple folders then only the first
        /// is returned.
        /// </summary>
        /// <param name="inventoryService"></param>
        /// <param name="userId"></param>
        /// <param name="path"></param>
        /// <returns>null if no folder matching the path was found</returns>
        public static InventoryFolderBase GetInventoryFolder(IInventoryService inventoryService, UUID userId, string path)
        {
            List<InventoryFolderBase> folders = GetInventoryFolders(inventoryService, userId, path);

            if (folders.Count != 0)
                return folders[0];
            else
                return null;
        }

        /// <summary>
        /// Get the inventory folders that match the path name.
        /// </summary>
        /// <param name="inventoryService"></param>
        /// <param name="userId"></param>
        /// <param name="path"></param>
        /// <returns>An empty list if no matching folders were found</returns>
        public static List<InventoryFolderBase> GetInventoryFolders(IInventoryService inventoryService, UUID userId, string path)
        {
            return InventoryArchiveUtils.FindFolderByPath(inventoryService, userId, path);
        }

        /// <summary>
        /// Get the inventory item that matches the path name.  If there are multiple items then only the first
        /// is returned.
        /// </summary>
        /// <param name="inventoryService"></param>
        /// <param name="userId"></param>
        /// <param name="path"></param>
        /// <returns>null if no item matching the path was found</returns>
        public static InventoryItemBase GetInventoryItem(IInventoryService inventoryService, UUID userId, string path)
        {
            return InventoryArchiveUtils.FindItemByPath(inventoryService, userId, path);
        }

        /// <summary>
        /// Get the inventory items that match the path name.
        /// </summary>
        /// <param name="inventoryService"></param>
        /// <param name="userId"></param>
        /// <param name="path"></param>
        /// <returns>An empty list if no matching items were found.</returns>
        public static List<InventoryItemBase> GetInventoryItems(IInventoryService inventoryService, UUID userId, string path)
        {
            return InventoryArchiveUtils.FindItemsByPath(inventoryService, userId, path);
        }
    }
}