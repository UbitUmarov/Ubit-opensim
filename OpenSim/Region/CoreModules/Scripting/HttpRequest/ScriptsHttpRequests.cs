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
using System.IO;
using System.Net;
using System.Net.Security;
using System.Text;
using System.Threading;
using System.Security.Cryptography.X509Certificates;
using Nini.Config;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Framework.Servers;
using OpenSim.Framework.Servers.HttpServer;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Region.Framework.Scenes;

/*****************************************************
 *
 * ScriptsHttpRequests
 *
 * Implements the llHttpRequest and http_response
 * callback.
 *
 * Some stuff was already in LSLLongCmdHandler, and then
 * there was this file with a stub class in it.  So,
 * I am moving some of the objects and functions out of
 * LSLLongCmdHandler, such as the HttpRequestClass, the
 * start and stop methods, and setting up pending and
 * completed queues.  These are processed in the
 * LSLLongCmdHandler polling loop.  Similiar to the
 * XMLRPCModule, since that seems to work.
 *
 * //TODO
 *
 * This probably needs some throttling mechanism but
 * it's wide open right now.  This applies to both
 * number of requests and data volume.
 *
 * Linden puts all kinds of header fields in the requests.
 * Not doing any of that:
 * User-Agent
 * X-SecondLife-Shard
 * X-SecondLife-Object-Name
 * X-SecondLife-Object-Key
 * X-SecondLife-Region
 * X-SecondLife-Local-Position
 * X-SecondLife-Local-Velocity
 * X-SecondLife-Local-Rotation
 * X-SecondLife-Owner-Name
 * X-SecondLife-Owner-Key
 *
 * HTTPS support
 *
 * Configurable timeout?
 * Configurable max response size?
 * Configurable
 *
 * **************************************************/

namespace OpenSim.Region.CoreModules.Scripting.HttpRequest
{
    public class HttpRequestModule : IRegionModule, IHttpRequestModule
    {
        private object HttpListLock = new object();
        private int httpTimeout = 30000;
        private string m_name = "HttpScriptRequests";

        private string m_proxyurl = "";
        private string m_proxyexcepts = "";

        // <request id, HttpRequestClass>
        private Dictionary<UUID, HttpRequestClass> m_pendingRequests;
        private Scene m_scene;
        // private Queue<HttpRequestClass> rpcQueue = new Queue<HttpRequestClass>();

        public HttpRequestModule()
        {
            ServicePointManager.ServerCertificateValidationCallback +=ValidateServerCertificate;
        }

        public static bool ValidateServerCertificate(
            object sender,
            X509Certificate  certificate,
            X509Chain  chain,
            SslPolicyErrors  sslPolicyErrors)
        {
            HttpWebRequest Request = (HttpWebRequest)sender;

            if (Request.Headers.Get("NoVerifyCert") != null)
            {
                return true;
            }
            
            if ((((int)sslPolicyErrors) & ~4) != 0)
                return false;

            if (ServicePointManager.CertificatePolicy != null)
            {
                ServicePoint sp = Request.ServicePoint;
                return ServicePointManager.CertificatePolicy.CheckValidationResult (sp, certificate, Request, 0);
            }
            return true;
        }
        #region IHttpRequestModule Members

        public UUID MakeHttpRequest(string url, string parameters, string body)
        {
            return UUID.Zero;
        }

        public UUID StartHttpRequest(uint localID, UUID itemID, string url, List<string> parameters, Dictionary<string, string> headers, string body)
        {
            UUID reqID = UUID.Random();
            HttpRequestClass htc = new HttpRequestClass();

            // Partial implementation: support for parameter flags needed
            //   see http://wiki.secondlife.com/wiki/LlHTTPRequest
            //
            // Parameters are expected in {key, value, ... , key, value}
            if (parameters != null)
            {
                string[] parms = parameters.ToArray();
                for (int i = 0; i < parms.Length; i += 2)
                {
                    switch (Int32.Parse(parms[i]))
                    {
                        case (int)HttpRequestConstants.HTTP_METHOD:

                            htc.HttpMethod = parms[i + 1];
                            break;

                        case (int)HttpRequestConstants.HTTP_MIMETYPE:

                            htc.HttpMIMEType = parms[i + 1];
                            break;

                        case (int)HttpRequestConstants.HTTP_BODY_MAXLENGTH:

                            // TODO implement me
                            break;

                        case (int)HttpRequestConstants.HTTP_VERIFY_CERT:
                            htc.HttpVerifyCert = (int.Parse(parms[i + 1]) != 0);
                            break;
                    }
                }
            }

            htc.LocalID = localID;
            htc.ItemID = itemID;
            htc.Url = url;
            htc.ReqID = reqID;
            htc.HttpTimeout = httpTimeout;
            htc.OutboundBody = body;
            htc.ResponseHeaders = headers;
            htc.proxyurl = m_proxyurl;
            htc.proxyexcepts = m_proxyexcepts;

            lock (HttpListLock)
            {
                m_pendingRequests.Add(reqID, htc);
            }

            htc.Process();

            return reqID;
        }

        public void StopHttpRequest(uint m_localID, UUID m_itemID)
        {
            if (m_pendingRequests != null)
            {
                lock (HttpListLock)
                {
                    HttpRequestClass tmpReq;
                    if (m_pendingRequests.TryGetValue(m_itemID, out tmpReq))
                    {
                        tmpReq.Stop();
                        m_pendingRequests.Remove(m_itemID);
                    }
                }
            }
        }

        /*
        * TODO
        * Not sure how important ordering is is here - the next first
        * one completed in the list is returned, based soley on its list
        * position, not the order in which the request was started or
        * finished.  I thought about setting up a queue for this, but
        * it will need some refactoring and this works 'enough' right now
        */

        public IServiceRequest GetNextCompletedRequest()
        {
            lock (HttpListLock)
            {
                foreach (UUID luid in m_pendingRequests.Keys)
                {
                    HttpRequestClass tmpReq;

                    if (m_pendingRequests.TryGetValue(luid, out tmpReq))
                    {
                        if (tmpReq.Finished)
                        {
                            return tmpReq;
                        }
                    }
                }
            }
            return null;
        }

        public void RemoveCompletedRequest(UUID id)
        {
            lock (HttpListLock)
            {
                HttpRequestClass tmpReq;
                if (m_pendingRequests.TryGetValue(id, out tmpReq))
                {
                    tmpReq.Stop();
                    tmpReq = null;
                    m_pendingRequests.Remove(id);
                }
            }
        }

        #endregion

        #region IRegionModule Members

        public void Initialise(Scene scene, IConfigSource config)
        {
            m_scene = scene;

            m_scene.RegisterModuleInterface<IHttpRequestModule>(this);

            m_proxyurl = config.Configs["Startup"].GetString("HttpProxy");
            m_proxyexcepts = config.Configs["Startup"].GetString("HttpProxyExceptions");

            m_pendingRequests = new Dictionary<UUID, HttpRequestClass>();
        }

        public void PostInitialise()
        {
        }

        public void Close()
        {
        }

        public string Name
        {
            get { return m_name; }
        }

        public bool IsSharedModule
        {
            get { return true; }
        }

        #endregion
    }

    public class HttpRequestClass: IServiceRequest
    {
        // Constants for parameters
        // public const int HTTP_BODY_MAXLENGTH = 2;
        // public const int HTTP_METHOD = 0;
        // public const int HTTP_MIMETYPE = 1;
        // public const int HTTP_VERIFY_CERT = 3;
        private bool _finished;
        public bool Finished
        { 
            get { return _finished; }
        }
        // public int HttpBodyMaxLen = 2048; // not implemented

        // Parameter members and default values
        public string HttpMethod  = "GET";
        public string HttpMIMEType = "text/plain;charset=utf-8";
        public int HttpTimeout;
        public bool HttpVerifyCert = true;
        private Thread httpThread;

        // Request info
        private UUID _itemID;
        public UUID ItemID 
        {
            get { return _itemID; }
            set { _itemID = value; }
        }
        private uint _localID;
        public uint LocalID
        {
            get { return _localID; }
            set { _localID = value; }
        }
        public DateTime Next;
        public string proxyurl;
        public string proxyexcepts;
        public string OutboundBody;
        private UUID _reqID;
        public UUID ReqID 
        {
            get { return _reqID; }
            set { _reqID = value; }
        }
        public HttpWebRequest Request;
        public string ResponseBody;
        public List<string> ResponseMetadata;
        public Dictionary<string, string> ResponseHeaders;
        public int Status;
        public string Url;

        public void Process()
        {
            httpThread = new Thread(SendRequest);
            httpThread.Name = "HttpRequestThread";
            httpThread.Priority = ThreadPriority.BelowNormal;
            httpThread.IsBackground = true;
            _finished = false;
            httpThread.Start();
        }

        /*
         * TODO: More work on the response codes.  Right now
         * returning 200 for success or 499 for exception
         */

        public void SendRequest()
        {
            HttpWebResponse response = null;
            StringBuilder sb = new StringBuilder();
            byte[] buf = new byte[8192];
            string tempString = null;
            int count = 0;

            try
            {
                Request = (HttpWebRequest) WebRequest.Create(Url);
                Request.Method = HttpMethod;
                Request.ContentType = HttpMIMEType;

                if(!HttpVerifyCert)
                {
                    // We could hijack Connection Group Name to identify
                    // a desired security exception.  But at the moment we'll use a dummy header instead.
//                    Request.ConnectionGroupName = "NoVerify";
                    Request.Headers.Add("NoVerifyCert", "true");
                }
//                else
//                {
//                    Request.ConnectionGroupName="Verify";
//                }
                if (proxyurl != null && proxyurl.Length > 0) 
                {
                    if (proxyexcepts != null && proxyexcepts.Length > 0) 
                    {
                        string[] elist = proxyexcepts.Split(';');
                        Request.Proxy = new WebProxy(proxyurl, true, elist);
                    } 
                    else 
                    {
                        Request.Proxy = new WebProxy(proxyurl, true);
                    }
                }

                foreach (KeyValuePair<string, string> entry in ResponseHeaders)
                    if (entry.Key.ToLower().Equals("user-agent"))
                        Request.UserAgent = entry.Value;
                    else
                        Request.Headers[entry.Key] = entry.Value;

                // Encode outbound data
                if (OutboundBody.Length > 0) 
                {
                    byte[] data = Util.UTF8.GetBytes(OutboundBody);

                    Request.ContentLength = data.Length;
                    Stream bstream = Request.GetRequestStream();
                    bstream.Write(data, 0, data.Length);
                    bstream.Close();
                }

                Request.Timeout = HttpTimeout;
                try
                {
                    // execute the request
                    response = (HttpWebResponse) Request.GetResponse();
                }
                catch (WebException e)
                {
                    if (e.Status != WebExceptionStatus.ProtocolError)
                    {
                        throw;
                    }
                    response = (HttpWebResponse)e.Response;
                }

                Status = (int)response.StatusCode;

                Stream resStream = response.GetResponseStream();

                do
                {
                    // fill the buffer with data
                    count = resStream.Read(buf, 0, buf.Length);

                    // make sure we read some data
                    if (count != 0)
                    {
                        // translate from bytes to ASCII text
                        tempString = Util.UTF8.GetString(buf, 0, count);

                        // continue building the string
                        sb.Append(tempString);
                    }
                } while (count > 0); // any more data to read?

                ResponseBody = sb.ToString();
            }
            catch (Exception e)
            {
                Status = (int)OSHttpStatusCode.ClientErrorJoker;
                ResponseBody = e.Message;

                _finished = true;
                return;
            }
            finally
            {
                if (response != null)
                    response.Close();
            }

            _finished = true;
        }

        public void Stop()
        {
            try
            {
                httpThread.Abort();
            }
            catch (Exception)
            {
            }
        }
    }
}
