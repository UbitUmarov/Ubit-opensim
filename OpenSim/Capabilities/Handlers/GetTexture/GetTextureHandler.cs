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
using System.Collections;
using System.Collections.Specialized;
using System.Drawing;
using System.Drawing.Imaging;
using System.Reflection;
using System.IO;
using System.Web;
using log4net;
using Nini.Config;
using OpenMetaverse;
using OpenMetaverse.StructuredData;
using OpenMetaverse.Imaging;
using OpenSim.Framework;
using OpenSim.Framework.Servers;
using OpenSim.Framework.Servers.HttpServer;
using OpenSim.Region.Framework.Interfaces;
using OpenSim.Services.Interfaces;
using Caps = OpenSim.Framework.Capabilities.Caps;

namespace OpenSim.Capabilities.Handlers
{
    public class GetTextureHandler : BaseStreamHandler
    {
        private static readonly ILog m_log =
            LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        private IAssetService m_assetService;

        public const string DefaultFormat = "x-j2c";

        // TODO: Change this to a config option
        const string REDIRECT_URL = null;

        public GetTextureHandler(string path, IAssetService assService) :
                base("GET", path)
        {
            m_assetService = assService;
        }

        public override byte[] Handle(string path, Stream request, IOSHttpRequest httpRequest, IOSHttpResponse httpResponse)
        {
            // Try to parse the texture ID from the request URL
            NameValueCollection query = HttpUtility.ParseQueryString(httpRequest.Url.Query);
            string textureStr = query.GetOne("texture_id");
            string format = query.GetOne("format");

            //m_log.DebugFormat("[GETTEXTURE]: called {0}", textureStr);

            if (m_assetService == null)
            {
                m_log.Error("[GETTEXTURE]: Cannot fetch texture " + textureStr + " without an asset service");
                httpResponse.StatusCode = (int)System.Net.HttpStatusCode.NotFound;
                return null;
            }

            UUID textureID;
            if (!String.IsNullOrEmpty(textureStr) && UUID.TryParse(textureStr, out textureID))
            {
//                m_log.DebugFormat("[GETTEXTURE]: Received request for texture id {0}", textureID);
                
                string[] formats;
                if (format != null && format != string.Empty)
                {
                    formats = new string[1] { format.ToLower() };
                }
                else
                {
                    formats = WebUtil.GetPreferredImageTypes(httpRequest.Headers.Get("Accept"));
                    if (formats.Length == 0)
                        formats = new string[1] { DefaultFormat }; // default

                }
                // OK, we have an array with preferred formats, possibly with only one entry

                httpResponse.StatusCode = (int)System.Net.HttpStatusCode.NotFound;
                foreach (string f in formats)
                {
                    if (FetchTexture(httpRequest, httpResponse, textureID, f))
                        break;
                }
            }
            else
            {
                m_log.Warn("[GETTEXTURE]: Failed to parse a texture_id from GetTexture request: " + httpRequest.Url);
            }

//            m_log.DebugFormat(
//                "[GETTEXTURE]: For texture {0} sending back response {1}, data length {2}",
//                textureID, httpResponse.StatusCode, httpResponse.ContentLength);

            httpResponse.Send();
            return null;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="httpRequest"></param>
        /// <param name="httpResponse"></param>
        /// <param name="textureID"></param>
        /// <param name="format"></param>
        /// <returns>False for "caller try another codec"; true otherwise</returns>
        private bool FetchTexture(IOSHttpRequest httpRequest, IOSHttpResponse httpResponse, UUID textureID, string format)
        {
//            m_log.DebugFormat("[GETTEXTURE]: {0} with requested format {1}", textureID, format);
            AssetBase texture;

            string fullID = textureID.ToString();
            if (format != DefaultFormat)
                fullID = fullID + "-" + format;

            if (!String.IsNullOrEmpty(REDIRECT_URL))
            {
                // Only try to fetch locally cached textures. Misses are redirected
                texture = m_assetService.GetCached(fullID);

                if (texture != null)
                {
                    if (texture.Type != (sbyte)AssetType.Texture)
                    {
                        httpResponse.StatusCode = (int)System.Net.HttpStatusCode.NotFound;
                        return true;
                    }
                    WriteTextureData(httpRequest, httpResponse, texture, format);
                }
                else
                {
                    string textureUrl = REDIRECT_URL + textureID.ToString();
                    m_log.Debug("[GETTEXTURE]: Redirecting texture request to " + textureUrl);
                    httpResponse.RedirectLocation = textureUrl;
                    return true;
                }
            }
            else // no redirect
            {
                // try the cache
                texture = m_assetService.GetCached(fullID);

                if (texture == null)
                {
                    //m_log.DebugFormat("[GETTEXTURE]: texture was not in the cache");

                    // Fetch locally or remotely. Misses return a 404
                    texture = m_assetService.Get(textureID.ToString());

                    if (texture != null)
                    {
                        if (texture.Type != (sbyte)AssetType.Texture)
                        {
                            httpResponse.StatusCode = (int)System.Net.HttpStatusCode.NotFound;
                            return true;
                        }
                        if (format == DefaultFormat)
                        {
                            WriteTextureData(httpRequest, httpResponse, texture, format);
                            return true;
                        }
                        else
                        {
                            AssetBase newTexture = new AssetBase(texture.ID + "-" + format, texture.Name, (sbyte)AssetType.Texture, texture.Metadata.CreatorID);
                            newTexture.Data = ConvertTextureData(texture, format);
                            if (newTexture.Data.Length == 0)
                                return false; // !!! Caller try another codec, please!

                            newTexture.Flags = AssetFlags.Collectable;
                            newTexture.Temporary = true;
                            m_assetService.Store(newTexture);
                            WriteTextureData(httpRequest, httpResponse, newTexture, format);
                            return true;
                        }
                    }
               }
               else // it was on the cache
               {
                   //m_log.DebugFormat("[GETTEXTURE]: texture was in the cache");
                   WriteTextureData(httpRequest, httpResponse, texture, format);
                   return true;
               }
            }

            // not found
//            m_log.Warn("[GETTEXTURE]: Texture " + textureID + " not found");
            httpResponse.StatusCode = (int)System.Net.HttpStatusCode.NotFound;
            return true;
        }

        private void WriteTextureData(IOSHttpRequest request, IOSHttpResponse response, AssetBase texture, string format)
        {
            string range = request.Headers.GetOne("Range");

            if (!String.IsNullOrEmpty(range)) // JP2's only
            {
                // Range request
                int start, end;
                if (TryParseRange(range, out start, out end))
                {

                    // Before clamping start make sure we can satisfy it in order to avoid
                    // sending back the last byte instead of an error status
                    if (start >= texture.Data.Length)
                    {
                        response.StatusCode = (int)System.Net.HttpStatusCode.RequestedRangeNotSatisfiable;
                    }
                    else
                    {
                        end = Utils.Clamp(end, 0, texture.Data.Length - 1);
                        start = Utils.Clamp(start, 0, end);
                        int len = end - start + 1;

                        //m_log.Debug("Serving " + start + " to " + end + " of " + texture.Data.Length + " bytes for texture " + texture.ID);

                        // Always return PartialContent, even if the range covered the entire data length
                        // We were accidentally sending back 404 before in this situation
                        // https://issues.apache.org/bugzilla/show_bug.cgi?id=51878 supports sending 206 even if the
                        // entire range is requested, and viewer 3.2.2 (and very probably earlier) seems fine with this.
                        response.StatusCode = (int)System.Net.HttpStatusCode.PartialContent;

                        response.ContentLength = len;
                        response.ContentType = texture.Metadata.ContentType;
                        response.AddHeader("Content-Range", String.Format("bytes {0}-{1}/{2}", start, end, texture.Data.Length));
    
                        response.Body.Write(texture.Data, start, len);
                    }
                }
                else
                {
                    m_log.Warn("[GETTEXTURE]: Malformed Range header: " + range);
                    response.StatusCode = (int)System.Net.HttpStatusCode.BadRequest;
                }
            }
            else // JP2's or other formats
            {
                // Full content request
                response.StatusCode = (int)System.Net.HttpStatusCode.OK;
                response.ContentLength = texture.Data.Length;
                if (format == DefaultFormat)
                    response.ContentType = texture.Metadata.ContentType;
                else
                    response.ContentType = "image/" + format;
                response.Body.Write(texture.Data, 0, texture.Data.Length);
            }

//            if (response.StatusCode < 200 || response.StatusCode > 299)
//                m_log.WarnFormat(
//                    "[GETTEXTURE]: For texture {0} requested range {1} responded {2} with content length {3} (actual {4})",
//                    texture.FullID, range, response.StatusCode, response.ContentLength, texture.Data.Length);
//            else
//                m_log.DebugFormat(
//                    "[GETTEXTURE]: For texture {0} requested range {1} responded {2} with content length {3} (actual {4})",
//                    texture.FullID, range, response.StatusCode, response.ContentLength, texture.Data.Length);
        }

        private bool TryParseRange(string header, out int start, out int end)
        {
            if (header.StartsWith("bytes="))
            {
                string[] rangeValues = header.Substring(6).Split('-');
                if (rangeValues.Length == 2)
                {
                    if (Int32.TryParse(rangeValues[0], out start) && Int32.TryParse(rangeValues[1], out end))
                        return true;
                }
            }

            start = end = 0;
            return false;
        }

        private byte[] ConvertTextureData(AssetBase texture, string format)
        {
            m_log.DebugFormat("[GETTEXTURE]: Converting texture {0} to {1}", texture.ID, format);
            byte[] data = new byte[0];

            MemoryStream imgstream = new MemoryStream();
            Bitmap mTexture = new Bitmap(1, 1);
            ManagedImage managedImage;
            Image image = (Image)mTexture;

            try
            {
                // Taking our jpeg2000 data, decoding it, then saving it to a byte array with regular data

                imgstream = new MemoryStream();

                // Decode image to System.Drawing.Image
                if (OpenJPEG.DecodeToImage(texture.Data, out managedImage, out image))
                {
                    // Save to bitmap
                    mTexture = new Bitmap(image);

                    EncoderParameters myEncoderParameters = new EncoderParameters();
                    myEncoderParameters.Param[0] = new EncoderParameter(Encoder.Quality, 95L);

                    // Save bitmap to stream
                    ImageCodecInfo codec = GetEncoderInfo("image/" + format);
                    if (codec != null)
                    {
                        mTexture.Save(imgstream, codec, myEncoderParameters);
                        // Write the stream to a byte array for output
                        data = imgstream.ToArray();
                    }
                    else
                        m_log.WarnFormat("[GETTEXTURE]: No such codec {0}", format);

                }
            }
            catch (Exception e)
            {
                m_log.WarnFormat("[GETTEXTURE]: Unable to convert texture {0} to {1}: {2}", texture.ID, format, e.Message);
            }
            finally
            {
                // Reclaim memory, these are unmanaged resources
                // If we encountered an exception, one or more of these will be null
                if (mTexture != null)
                    mTexture.Dispose();

                if (image != null)
                    image.Dispose();

                if (imgstream != null)
                {
                    imgstream.Close();
                    imgstream.Dispose();
                }
            }

            return data;
        }

        // From msdn
        private static ImageCodecInfo GetEncoderInfo(String mimeType)
        {
            ImageCodecInfo[] encoders;
            encoders = ImageCodecInfo.GetImageEncoders();
            for (int j = 0; j < encoders.Length; ++j)
            {
                if (encoders[j].MimeType == mimeType)
                    return encoders[j];
            }
            return null;
        }
    }
}