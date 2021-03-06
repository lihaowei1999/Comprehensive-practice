// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ioAV.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "ioAV.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
// @@protoc_insertion_point(includes)

namespace ioAV {

void protobuf_ShutdownFile_ioAV_2eproto() {
  delete Perception::default_instance_;
}

#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
void protobuf_AddDesc_ioAV_2eproto_impl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#else
void protobuf_AddDesc_ioAV_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_ioAV_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#endif
  Perception::default_instance_ = new Perception();
  Perception::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_ioAV_2eproto);
}

#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AddDesc_ioAV_2eproto_once_);
void protobuf_AddDesc_ioAV_2eproto() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AddDesc_ioAV_2eproto_once_,
                 &protobuf_AddDesc_ioAV_2eproto_impl);
}
#else
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_ioAV_2eproto {
  StaticDescriptorInitializer_ioAV_2eproto() {
    protobuf_AddDesc_ioAV_2eproto();
  }
} static_descriptor_initializer_ioAV_2eproto_;
#endif

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Perception::kTypeFieldNumber;
const int Perception::kTimestampFieldNumber;
const int Perception::kSyncFlagFieldNumber;
const int Perception::kBodyFieldNumber;
const int Perception::kCheckCodeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Perception::Perception()
  : ::google::protobuf::MessageLite(), _arena_ptr_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:ioAV.Perception)
}

void Perception::InitAsDefaultInstance() {
  _is_default_instance_ = true;
}

Perception::Perception(const Perception& from)
  : ::google::protobuf::MessageLite(),
    _arena_ptr_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:ioAV.Perception)
}

void Perception::SharedCtor() {
    _is_default_instance_ = false;
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  type_ = 0;
  timestamp_ = GOOGLE_ULONGLONG(0);
  sync_flag_ = 0;
  body_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  checkcode_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

Perception::~Perception() {
  // @@protoc_insertion_point(destructor:ioAV.Perception)
  SharedDtor();
}

void Perception::SharedDtor() {
  body_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  checkcode_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  if (this != &default_instance()) {
  #else
  if (this != default_instance_) {
  #endif
  }
}

void Perception::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const Perception& Perception::default_instance() {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  protobuf_AddDesc_ioAV_2eproto();
#else
  if (default_instance_ == NULL) protobuf_AddDesc_ioAV_2eproto();
#endif
  return *default_instance_;
}

Perception* Perception::default_instance_ = NULL;

Perception* Perception::New(::google::protobuf::Arena* arena) const {
  Perception* n = new Perception;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Perception::Clear() {
// @@protoc_insertion_point(message_clear_start:ioAV.Perception)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(Perception, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<Perception*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  ZR_(timestamp_, sync_flag_);
  body_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  checkcode_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());

#undef ZR_HELPER_
#undef ZR_

}

bool Perception::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:ioAV.Perception)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional int32 type = 1;
      case 1: {
        if (tag == 8) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &type_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_timestamp;
        break;
      }

      // optional fixed64 timestamp = 2;
      case 2: {
        if (tag == 17) {
         parse_timestamp:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_FIXED64>(
                 input, &timestamp_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(24)) goto parse_sync_flag;
        break;
      }

      // optional int32 sync_flag = 3;
      case 3: {
        if (tag == 24) {
         parse_sync_flag:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &sync_flag_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(34)) goto parse_body;
        break;
      }

      // optional bytes body = 4;
      case 4: {
        if (tag == 34) {
         parse_body:
          DO_(::google::protobuf::internal::WireFormatLite::ReadBytes(
                input, this->mutable_body()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(42)) goto parse_checkCode;
        break;
      }

      // optional string checkCode = 5;
      case 5: {
        if (tag == 42) {
         parse_checkCode:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_checkcode()));
          DO_(::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
            this->checkcode().data(), this->checkcode().length(),
            ::google::protobuf::internal::WireFormatLite::PARSE,
            "ioAV.Perception.checkCode"));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:ioAV.Perception)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:ioAV.Perception)
  return false;
#undef DO_
}

void Perception::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:ioAV.Perception)
  // optional int32 type = 1;
  if (this->type() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(1, this->type(), output);
  }

  // optional fixed64 timestamp = 2;
  if (this->timestamp() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFixed64(2, this->timestamp(), output);
  }

  // optional int32 sync_flag = 3;
  if (this->sync_flag() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->sync_flag(), output);
  }

  // optional bytes body = 4;
  if (this->body().size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBytesMaybeAliased(
      4, this->body(), output);
  }

  // optional string checkCode = 5;
  if (this->checkcode().size() > 0) {
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
      this->checkcode().data(), this->checkcode().length(),
      ::google::protobuf::internal::WireFormatLite::SERIALIZE,
      "ioAV.Perception.checkCode");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      5, this->checkcode(), output);
  }

  // @@protoc_insertion_point(serialize_end:ioAV.Perception)
}

int Perception::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:ioAV.Perception)
  int total_size = 0;

  // optional int32 type = 1;
  if (this->type() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->type());
  }

  // optional fixed64 timestamp = 2;
  if (this->timestamp() != 0) {
    total_size += 1 + 8;
  }

  // optional int32 sync_flag = 3;
  if (this->sync_flag() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->sync_flag());
  }

  // optional bytes body = 4;
  if (this->body().size() > 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::BytesSize(
        this->body());
  }

  // optional string checkCode = 5;
  if (this->checkcode().size() > 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->checkcode());
  }

  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Perception::CheckTypeAndMergeFrom(
    const ::google::protobuf::MessageLite& from) {
  MergeFrom(*::google::protobuf::down_cast<const Perception*>(&from));
}

void Perception::MergeFrom(const Perception& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ioAV.Perception)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from.type() != 0) {
    set_type(from.type());
  }
  if (from.timestamp() != 0) {
    set_timestamp(from.timestamp());
  }
  if (from.sync_flag() != 0) {
    set_sync_flag(from.sync_flag());
  }
  if (from.body().size() > 0) {

    body_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.body_);
  }
  if (from.checkcode().size() > 0) {

    checkcode_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.checkcode_);
  }
}

void Perception::CopyFrom(const Perception& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ioAV.Perception)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Perception::IsInitialized() const {

  return true;
}

void Perception::Swap(Perception* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Perception::InternalSwap(Perception* other) {
  std::swap(type_, other->type_);
  std::swap(timestamp_, other->timestamp_);
  std::swap(sync_flag_, other->sync_flag_);
  body_.Swap(&other->body_);
  checkcode_.Swap(&other->checkcode_);
  _unknown_fields_.Swap(&other->_unknown_fields_);
  std::swap(_cached_size_, other->_cached_size_);
}

::std::string Perception::GetTypeName() const {
  return "ioAV.Perception";
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Perception

// optional int32 type = 1;
void Perception::clear_type() {
  type_ = 0;
}
 ::google::protobuf::int32 Perception::type() const {
  // @@protoc_insertion_point(field_get:ioAV.Perception.type)
  return type_;
}
 void Perception::set_type(::google::protobuf::int32 value) {
  
  type_ = value;
  // @@protoc_insertion_point(field_set:ioAV.Perception.type)
}

// optional fixed64 timestamp = 2;
void Perception::clear_timestamp() {
  timestamp_ = GOOGLE_ULONGLONG(0);
}
 ::google::protobuf::uint64 Perception::timestamp() const {
  // @@protoc_insertion_point(field_get:ioAV.Perception.timestamp)
  return timestamp_;
}
 void Perception::set_timestamp(::google::protobuf::uint64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:ioAV.Perception.timestamp)
}

// optional int32 sync_flag = 3;
void Perception::clear_sync_flag() {
  sync_flag_ = 0;
}
 ::google::protobuf::int32 Perception::sync_flag() const {
  // @@protoc_insertion_point(field_get:ioAV.Perception.sync_flag)
  return sync_flag_;
}
 void Perception::set_sync_flag(::google::protobuf::int32 value) {
  
  sync_flag_ = value;
  // @@protoc_insertion_point(field_set:ioAV.Perception.sync_flag)
}

// optional bytes body = 4;
void Perception::clear_body() {
  body_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 const ::std::string& Perception::body() const {
  // @@protoc_insertion_point(field_get:ioAV.Perception.body)
  return body_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Perception::set_body(const ::std::string& value) {
  
  body_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:ioAV.Perception.body)
}
 void Perception::set_body(const char* value) {
  
  body_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:ioAV.Perception.body)
}
 void Perception::set_body(const void* value, size_t size) {
  
  body_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:ioAV.Perception.body)
}
 ::std::string* Perception::mutable_body() {
  
  // @@protoc_insertion_point(field_mutable:ioAV.Perception.body)
  return body_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* Perception::release_body() {
  // @@protoc_insertion_point(field_release:ioAV.Perception.body)
  
  return body_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Perception::set_allocated_body(::std::string* body) {
  if (body != NULL) {
    
  } else {
    
  }
  body_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), body);
  // @@protoc_insertion_point(field_set_allocated:ioAV.Perception.body)
}

// optional string checkCode = 5;
void Perception::clear_checkcode() {
  checkcode_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 const ::std::string& Perception::checkcode() const {
  // @@protoc_insertion_point(field_get:ioAV.Perception.checkCode)
  return checkcode_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Perception::set_checkcode(const ::std::string& value) {
  
  checkcode_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:ioAV.Perception.checkCode)
}
 void Perception::set_checkcode(const char* value) {
  
  checkcode_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:ioAV.Perception.checkCode)
}
 void Perception::set_checkcode(const char* value, size_t size) {
  
  checkcode_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:ioAV.Perception.checkCode)
}
 ::std::string* Perception::mutable_checkcode() {
  
  // @@protoc_insertion_point(field_mutable:ioAV.Perception.checkCode)
  return checkcode_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* Perception::release_checkcode() {
  // @@protoc_insertion_point(field_release:ioAV.Perception.checkCode)
  
  return checkcode_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Perception::set_allocated_checkcode(::std::string* checkcode) {
  if (checkcode != NULL) {
    
  } else {
    
  }
  checkcode_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), checkcode);
  // @@protoc_insertion_point(field_set_allocated:ioAV.Perception.checkCode)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace ioAV

// @@protoc_insertion_point(global_scope)
