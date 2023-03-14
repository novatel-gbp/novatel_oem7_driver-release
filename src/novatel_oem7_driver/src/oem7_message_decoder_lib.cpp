////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#include <oem7_message_decoder_lib.hpp>


#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <decoders/novatel/framer.hpp>

#include <memory>

#include <iostream>



namespace
{
  // Versioning: reflects underlying EDIE version
  static const novatel_oem7::version_element_t VERSION_MAJOR  = 10;
  static const novatel_oem7::version_element_t VERSION_MINOR  = 2;
  static const novatel_oem7::version_element_t VERSION_SPECIAL= 0;
}


namespace novatel_oem7
{
  /**
   * A wrapper for BaseMessageData
   * Hides EDIE accessors / data that need not be exposed (yet)
   */
  class Oem7RawMessage: public Oem7RawMessageIf
  {
    boost::shared_ptr<BaseMessageData> bmd_; ///< binary message obtained from receiver


  public:
    Oem7RawMessage(BaseMessageData* raw_bmd):
      bmd_(raw_bmd)
    {
    }

    /**
     * @return type, Log, Response, etc.
     */
    Oem7MessageType   getMessageType() const
    {
      if(bmd_->getMessageType())
      {
        return OEM7MSGTYPE_RSP;
      }
      else
      {
        return OEM7MSGTYPE_LOG;
      }
    }

    /**
     * @return format, Binary or ASCII
     */
    Oem7MessageFormat  getMessageFormat() const
    {
      switch(bmd_->getMessageFormat())
      {
        case MESSAGE_BINARY:               return OEM7MSGFMT_BINARY;
        case MESSAGE_SHORT_HEADER_BINARY:  return OEM7MSGFMT_SHORTBINARY;
        case MESSAGE_ASCII:                return OEM7MSGFMT_ASCII;
        case MESSAGE_ABB_ASCII:            return OEM7MSGFMT_ABASCII;
        default:                           return OEM7MSGFMT_UNKNOWN;
      }
    }

    /**
     * @return Oem7 message ID
     */
    int getMessageId() const
    {
      return bmd_->getMessageID();
    }

    /**
     * @return message data blog
     */
    const uint8_t* getMessageData(size_t offset) const
    {
      return reinterpret_cast<uint8_t*>(&bmd_->getMessageData()[offset]);
    }

    /**
     * @return length of message data
     */
    size_t getMessageDataLength() const
    {
      return bmd_->getMessageLength();
    }

  };


/***
 * Adapter between Decoder user, and the 'Stream' interface required by EDIE's standard decoder.
 */
class InputStream: public InputStreamInterface
{
  Oem7MessageDecoderLibUserIf* user_; ///< Decoder's user.

  public:
    InputStream(Oem7MessageDecoderLibUserIf* user):
      user_(user)
    {
    }
  
    /***
     * Called by EDIE to read bytes; refer to EDIE documentation
     */
    StreamReadStatus
    ReadData(ReadDataStructure& read_data)
    {
      size_t rlen = 0;
      bool ok = user_->read(boost::asio::buffer(read_data.cData, read_data.uiDataSize), rlen);
  
      StreamReadStatus st;
      st.bEOS = !ok;
      st.uiCurrentStreamRead = rlen;
  
      return st;
    }


    // Default empty implementation
    virtual StreamReadStatus ReadLine(std::string&) { return StreamReadStatus(); };

    virtual std::string GetFileExtension(){return NULL;};
    virtual void RegisterCallBack(NovatelParser*){};
    virtual void SetTimeOut(DOUBLE){};
    virtual void EnableCallBack(BOOL){};
    virtual void Reset(std::streamoff, std::ios_base::seekdir){};
    virtual BOOL IsCallBackEnable(){return FALSE;};
  };



/**
 * Oem7 Decoder Library implementation, wrapping and hiding EDIE interfaces.
 */
class Oem7MessageDecoderLib: public Oem7MessageDecoderLibIf
{
  Oem7MessageDecoderLibUserIf* user_;
  
  boost::shared_ptr<InputStream>     input_stream_; ///< EDIE input stream; refer to EDIE documentation
  boost::shared_ptr<Framer> framer_;   ///< EDIE standard framer
  
public:
  Oem7MessageDecoderLib(Oem7MessageDecoderLibUserIf* user):
    user_(user)
  {
    input_stream_ = boost::make_shared<InputStream>(user);
    framer_       = boost::make_shared<Framer>(input_stream_.get());

    framer_->EnableUnknownData(TRUE);
    framer_->SetBMDOutput(FLATTEN);
  }
  
  /**
   * Read a complete Oem7 message from EDIE
   */
  virtual bool readMessage(boost::shared_ptr<Oem7RawMessageIf>& msg)
  {
    BaseMessageData* raw_bmd = NULL;
    StreamReadStatus status = framer_->ReadMessage(&raw_bmd);
    if(raw_bmd)
    {
      msg = boost::make_shared<Oem7RawMessage>(raw_bmd);
    }
  
    // EOS: No more data is available from EDIE, e.g. EOF reached when reading from file, socket connection broken, etc.
    return !status.bEOS;
    // Presumably, when no message is reported, EOS is signaled. We can't handle this reliably here, let the User deal with this.
  }
  
};

/**
 * Factory function
 */
boost::shared_ptr<Oem7MessageDecoderLibIf>
GetOem7MessageDecoder(Oem7MessageDecoderLibUserIf* user)
{
  boost::shared_ptr<Oem7MessageDecoderLib> dec(new Oem7MessageDecoderLib(user));
  return dec;
}

void
GetOem7MessageDecoderLibVersion(version_element_t& major, version_element_t& minor, version_element_t& spec)
{
  major = VERSION_MAJOR;
  minor = VERSION_MINOR;
  spec  = VERSION_SPECIAL;
}

}
