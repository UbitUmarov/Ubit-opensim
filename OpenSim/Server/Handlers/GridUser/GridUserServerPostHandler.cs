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

using Nini.Config;
using log4net;
using System;
using System.Reflection;
using System.IO;
using System.Net;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml;
using System.Xml.Serialization;
using System.Collections.Generic;
using OpenSim.Server.Base;
using OpenSim.Services.Interfaces;
using OpenSim.Framework;
using OpenSim.Framework.Servers.HttpServer;
using OpenMetaverse;

namespace OpenSim.Server.Handlers.GridUser
{
    public class GridUserServerPostHandler : BaseStreamHandler
    {
        private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private IGridUserService m_GridUserService;

        public GridUserServerPostHandler(IGridUserService service) :
                base("POST", "/griduser")
        {
            m_GridUserService = service;
        }

        public override byte[] Handle(string path, Stream requestData,
                IOSHttpRequest httpRequest, IOSHttpResponse httpResponse)
        {
            StreamReader sr = new StreamReader(requestData);
            string body = sr.ReadToEnd();
            sr.Close();
            body = body.Trim();

            //m_log.DebugFormat("[XXX]: query String: {0}", body);
            string method = string.Empty;
            try
            {
                Dictionary<string, object> request =
                        ServerUtils.ParseQueryString(body);

                if (!request.ContainsKey("METHOD"))
                    return FailureResult();

                method = request["METHOD"].ToString();

                switch (method)
                {
                    case "loggedin":
                        return LoggedIn(request);
                    case "loggedout":
                        return LoggedOut(request);
                    case "sethome":
                        return SetHome(request);
                    case "setposition":
                        return SetPosition(request);
                    case "getgriduserinfo":
                        return GetGridUserInfo(request);
                }
                m_log.DebugFormat("[GRID USER HANDLER]: unknown method request: {0}", method);
            }
            catch (Exception e)
            {
                m_log.DebugFormat("[GRID USER HANDLER]: Exception in method {0}: {1}", method, e);
            }

            return FailureResult();

        }

        byte[] LoggedIn(Dictionary<string, object> request)
        {
            string user = String.Empty;

            if (!request.ContainsKey("UserID"))
                return FailureResult();

            user = request["UserID"].ToString();

            GridUserInfo guinfo = m_GridUserService.LoggedIn(user);

            Dictionary<string, object> result = new Dictionary<string, object>();
            result["result"] = guinfo.ToKeyValuePairs();

            string xmlString = ServerUtils.BuildXmlResponse(result);
            //m_log.DebugFormat("[GRID USER HANDLER]: resp string: {0}", xmlString);
            UTF8Encoding encoding = new UTF8Encoding();
            return encoding.GetBytes(xmlString);

        }

        byte[] LoggedOut(Dictionary<string, object> request)
        {
            string userID = string.Empty;
            UUID regionID = UUID.Zero;
            Vector3 position = Vector3.Zero;
            Vector3 lookat = Vector3.Zero;

            if (!UnpackArgs(request, out userID, out regionID, out position, out lookat))
                return FailureResult();

            if (m_GridUserService.LoggedOut(userID, UUID.Zero, regionID, position, lookat))
                return SuccessResult();

            return FailureResult();
        }

        byte[] SetHome(Dictionary<string, object> request)
        {
            string user = string.Empty;
            UUID region = UUID.Zero;
            Vector3 position = new Vector3(128, 128, 70);
            Vector3 look = Vector3.Zero;

            if (!UnpackArgs(request, out user, out region, out position, out look))
                return FailureResult();

            if (m_GridUserService.SetHome(user, region, position, look))
                return SuccessResult();

            return FailureResult();
        }

        byte[] SetPosition(Dictionary<string, object> request)
        {
            string user = string.Empty;
            UUID region = UUID.Zero;
            Vector3 position = new Vector3(128, 128, 70);
            Vector3 look = Vector3.Zero;

            if (!request.ContainsKey("UserID") || !request.ContainsKey("RegionID"))
                return FailureResult();

            if (!UnpackArgs(request, out user, out region, out position, out look))
                return FailureResult();

            if (m_GridUserService.SetLastPosition(user, UUID.Zero, region, position, look))
                return SuccessResult();

            return FailureResult();
        }

        byte[] GetGridUserInfo(Dictionary<string, object> request)
        {
            string user = String.Empty;

            if (!request.ContainsKey("UserID"))
                return FailureResult();

            user = request["UserID"].ToString();

            GridUserInfo guinfo = m_GridUserService.GetGridUserInfo(user);

            Dictionary<string, object> result = new Dictionary<string, object>();
            result["result"] = guinfo.ToKeyValuePairs();

            string xmlString = ServerUtils.BuildXmlResponse(result);
            //m_log.DebugFormat("[GRID USER HANDLER]: resp string: {0}", xmlString);
            UTF8Encoding encoding = new UTF8Encoding();
            return encoding.GetBytes(xmlString);

        }

        private bool UnpackArgs(Dictionary<string, object> request, out string user, out UUID region, out Vector3 position, out Vector3 lookAt)
        {
            user = string.Empty;
            region = UUID.Zero;
            position = new Vector3(128, 128, 70);
            lookAt = Vector3.Zero;

            if (!request.ContainsKey("UserID") || !request.ContainsKey("RegionID"))
                return false;

            user = request["UserID"].ToString();

            if (!UUID.TryParse(request["RegionID"].ToString(), out region))
                return false;

            if (request.ContainsKey("Position"))
                Vector3.TryParse(request["Position"].ToString(), out position);

            if (request.ContainsKey("LookAt"))
                Vector3.TryParse(request["LookAt"].ToString(), out lookAt);

            return true;
        }


        private byte[] SuccessResult()
        {
            XmlDocument doc = new XmlDocument();

            XmlNode xmlnode = doc.CreateNode(XmlNodeType.XmlDeclaration,
                    "", "");

            doc.AppendChild(xmlnode);

            XmlElement rootElement = doc.CreateElement("", "ServerResponse",
                    "");

            doc.AppendChild(rootElement);

            XmlElement result = doc.CreateElement("", "result", "");
            result.AppendChild(doc.CreateTextNode("Success"));

            rootElement.AppendChild(result);

            return DocToBytes(doc);
        }

        private byte[] FailureResult()
        {
            XmlDocument doc = new XmlDocument();

            XmlNode xmlnode = doc.CreateNode(XmlNodeType.XmlDeclaration,
                    "", "");

            doc.AppendChild(xmlnode);

            XmlElement rootElement = doc.CreateElement("", "ServerResponse",
                    "");

            doc.AppendChild(rootElement);

            XmlElement result = doc.CreateElement("", "result", "");
            result.AppendChild(doc.CreateTextNode("Failure"));

            rootElement.AppendChild(result);

            return DocToBytes(doc);
        }

        private byte[] DocToBytes(XmlDocument doc)
        {
            MemoryStream ms = new MemoryStream();
            XmlTextWriter xw = new XmlTextWriter(ms, null);
            xw.Formatting = Formatting.Indented;
            doc.WriteTo(xw);
            xw.Flush();

            return ms.ToArray();
        }


    }
}
