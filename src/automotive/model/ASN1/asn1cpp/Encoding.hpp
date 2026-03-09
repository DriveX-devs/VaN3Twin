#ifndef ASN1CPP_ENCODING_HEADER_FILE
#define ASN1CPP_ENCODING_HEADER_FILE

#include <string>

#include "asn_application.h"

#include "Utils.hpp"

namespace asn1cpp {
    template <typename T>
    class Seq;

    namespace Impl {
        inline int fill(const void * buffer, size_t size, void * appKey) {
            std::string * str = static_cast<std::string*>(appKey);
            if (!str)
                return -1;
            
            str->append((const char*)buffer, size);

            return 0;
        }
    }

    namespace ber {
        /**
         * @ingroup API
         * @brief Encodes an asn1cpp wrapper to an std::string.
         *
         * \sa decode(m, T)
         */
        template <typename T, typename = typename std::enable_if<is_asn1_wrapper<T>::value>::type>
        std::string encode(const T & m) {
            if (!m) return {};
            std::string retval;
            const auto er = der_encode(m.getTypeDescriptor(), (void*)(&*m), Impl::fill, &retval);
            return er.encoded < 0 ? std::string() : retval;
        }

        template <typename T>
        Seq<T> decode(asn_TYPE_descriptor_t * def, const std::string & buffer) {
            if (buffer.size() == 0) return Seq<T>();

            T * m = nullptr;
            const auto dr = ber_decode(0, def, (void**)&m, buffer.data(), buffer.size());

            if (dr.code != RC_OK) {
                def->op->free_struct(def, m , ASFM_FREE_EVERYTHING);
                return Seq<T>();
            }

            return Seq<T>(def, m);
        }
    }

    namespace xer {
        /**
         * @ingroup API
         * @brief Encodes an asn1cpp wrapper to an std::string.
         *
         * \sa decode(m, T)
         */
        template <typename T, typename = typename std::enable_if<is_asn1_wrapper<T>::value>::type>
        std::string encode(const T & m) {
            if (!m) return {};
            std::string retval;
            const auto er = xer_encode(m.getTypeDescriptor(), (void*)(&*m), XER_F_BASIC, Impl::fill, &retval);
            return er.encoded < 0 ? std::string() : retval;
        }
    }

    namespace uper {
    /**
         * @ingroup API
         * @brief Encodes an asn1cpp wrapper to an std::string.
         *
         * \sa decode(m, T)
         */
    template <typename T, typename = typename std::enable_if<is_asn1_wrapper<T>::value>::type>
    std::string encode(const T & m) {
      if (!m) return {};
      std::string retval;
      const auto er = uper_encode(m.getTypeDescriptor(), nullptr, (void*)(&*m), Impl::fill, &retval);
      return er.encoded < 0 ? std::string() : retval;
    }

    template <typename T>
    Seq<T> decode(asn_TYPE_descriptor_t * def, const std::string & buffer) {
      if (buffer.size() == 0) return Seq<T>();

      T * m = nullptr;
      const auto dr = uper_decode_complete(0, def, (void**)&m, buffer.data(), buffer.size());

      if (dr.code != RC_OK) {
          def->op->free_struct(def, m, ASFM_FREE_EVERYTHING);
          return Seq<T>();
        }

      return Seq<T>(def, m);
    }
    }

    namespace oer {
    template <typename T, typename = typename std::enable_if<is_asn1_wrapper<T>::value>::type>
    std::string encode(const T & m) {
      if (!m) return {};
      std::string retval;
      const auto er = oer_encode(m.getTypeDescriptor(), (void*)(&*m), Impl::fill, &retval);
      return er.encoded < 0 ? std::string() : retval;
    }

    template <typename T>
    Seq<T> decode(asn_TYPE_descriptor_t * def, const std::string & buffer) {
      if (buffer.size() == 0) return Seq<T>();

      T * m = nullptr;
      const auto dr = oer_decode(0, def, (void**)&m, buffer.data(), buffer.size());

      if (dr.code != RC_OK) {
          def->op->free_struct(def, m, ASFM_FREE_EVERYTHING);
          return Seq<T>();
        }

      return Seq<T>(def, m);
    }
    }
}

/**
 * @page EncodingPage Encoding and Decoding with asn1cpp
 *
 * All functions described here can be found in the \ref API group.
 *
 * Note that in asn1cpp some functions are macros due to the need to
 * automatically access the asn1c type descriptors from type names. This may
 * not play nice with your autocompletion features.
 *
 * Encoding and Decoding
 * ---------------------
 *
 * Encoding and decoding is done through the `encode` and `decode` functions.
 * Accessing a specific type of encoding/decoding is done through namespaces.
 * The `ber` namespace offers access to the `encode` and `decode` functions for
 * BER encoding, the `uper` namespaces does the same for UPER, and so on.
 *
 * Encoding and decoding an asn1cpp wrapper (be it a asn1cpp::Seq or a
 * asn1cpp::View) is extremely simple. For example:
 *
 * ```
 * auto s = asn1cpp::makeSeq(MyAsnType);
 * // set fields through setField functions...
 *
 * // We encode the Seq, and we obtain an std::string
 * auto enc = asn1cpp::ber::encode(s);
 *
 * // ...
 * // Somewhere else, we decode
 * auto decoded = asn1cpp::ber::decode(enc, MyAsnType);
 * ```
 *
 * Note that only decoding needs the type specification, since the asn1cpp
 * wrappers already know the type of their wrapped value.
 */


    /**
 * @def decode(m, T)
 * @ingroup API
 * @brief Decodes an std::string to an asn1cpp wrapper for the specified type.
 *
 * This macro must be prefixed with both the asn1cpp namespace and the
 * namespace of the encoding you want (ber, uper, ...).
 *
 * This macro allows specifying the name of the underlying asn1c type, and it
 * will automatically find the appropriate asn1c type descriptor for you.
 */
#define decodeASN(m, T) \
    decode<T>(&ASN1CPP_ASN1C_DEF(T), m)

#endif
