#include "security.h"
#include "ns3/log.h"
#include "ns3/SequenceOf.hpp"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include <openssl/sha.h>
#include <vector>
#include <openssl/obj_mac.h>
#include <openssl/err.h>
#include <openssl/bn.h>
#include <iostream>
#include <string>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <openssl/ec.h>

extern "C" {
#include "ns3/CAM.h"
#include "ns3/Ieee1609Dot2Data.h"
#include "ns3/Ieee1609Dot2Content.h"
#include "ns3/EtsiTs103097Data.h"
#include "ns3/CertificateBase.h"
#
}

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Security");

TypeId
Security::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Security").SetParent<Object> ().AddConstructor<Security> ();
  return tid;
}

Security::~Security ()
{
  if (m_ecKey != nullptr)
    {
      EC_KEY_free(m_ecKey);
    }
  NS_LOG_FUNCTION (this);
}

Security::Security ()
{
  m_eventCleaner = Simulator::Schedule(MilliSeconds (1000),&Security::mapCleaner,this);
  m_ecKey = nullptr;
  m_protocolVersion = 3;
  m_hashId = HashAlgorithm_sha256;
  m_psid = 36;
  m_psid2 = 37;
  m_generationTime = 629405121939330;
  m_digest = "1e7d7e81";
  m_version = 3;
  m_type = CertificateType_explicit;
  m_issuer = "00000000";
  m_id = 0;
  m_cracaId = "000";
  m_crlSeries = 0;
  m_validityPeriod_start = 628830005;
  m_validityPeriod_duration = 168;
  m_bitmapSsp = "010000";
  m_bitmapSsp2 = "01901a25";
  m_p256_x_only_Cert = "c35458670a819720adf1be43c782c2c0";
  m_SsigCert = "z35458670a819720adf1be43c782c2c0";

}



std::vector<unsigned char>
Security::hexStringToBytes (const std::string &hex)
{
  std::vector<unsigned char> bytes;
  for (unsigned int i = 0; i < hex.length (); i += 2)
    {
      std::string byteString = hex.substr (i, 2);
      unsigned char byte = static_cast<unsigned char> (strtol (byteString.c_str (), nullptr, 16));
      bytes.push_back (byte);
    }
  return bytes;
}

void Security::mapCleaner ()
{
  uint64_t now = Simulator::Now().GetMilliSeconds();
  for (auto it = m_receivedCertificates.begin(); it != m_receivedCertificates.end(); ) {
      if (it->first < now - 1000) {
          it = m_receivedCertificates.erase(it);
        } else {
          ++it;
        }
    }
  m_eventCleaner = Simulator::Schedule(MilliSeconds(1000), &Security::mapCleaner, this);
}



void
Security::computeSHA256 (const std::vector<unsigned char> &data,
                         unsigned char hash[SHA256_DIGEST_LENGTH])
{
  SHA256_CTX sha256;
  SHA256_Init (&sha256);
  SHA256_Update (&sha256, data.data (), data.size ());
  SHA256_Final (hash, &sha256);
}

std::vector<unsigned char>
Security::concatenateHashes (const unsigned char hash1[SHA256_DIGEST_LENGTH],
                             const unsigned char hash2[SHA256_DIGEST_LENGTH])
{
  std::vector<unsigned char> concatenatedHashes;
  concatenatedHashes.insert (concatenatedHashes.end (), hash1, hash1 + SHA256_DIGEST_LENGTH);
  concatenatedHashes.insert (concatenatedHashes.end (), hash2, hash2 + SHA256_DIGEST_LENGTH);
  return concatenatedHashes;
}

void
Security::print_openssl_error ()
{
  char buffer[120];
  unsigned long error = ERR_get_error ();
  ERR_error_string_n (error, buffer, sizeof (buffer));
  std::cerr << buffer << std::endl;
}


Security::GNpublicKey
Security::generateECKeyPair ()
{

  EC_KEY *ec_key = EC_KEY_new_by_curve_name (NID_X9_62_prime256v1);
  if (!ec_key)
    {
      std::cerr << "Error creating EC_KEY object" << std::endl;
      print_openssl_error ();
      return {};
    }

  if (!EC_KEY_generate_key (ec_key))
    {
      std::cerr << "Error generating EC key pair" << std::endl;
      print_openssl_error ();
      EC_KEY_free (ec_key);
      return{};
    }


  m_ecKey = EC_KEY_dup(ec_key);

  // Get the public key in hex form
  const EC_POINT *pub_key_point = EC_KEY_get0_public_key (ec_key);
  if (!pub_key_point)
    {
      std::cerr << "Error getting public key" << std::endl;
      print_openssl_error ();
      EC_KEY_free (ec_key);
      return {};
    }

  BN_CTX *ctx = BN_CTX_new ();
  if (!ctx)
    {
      std::cerr << "Error creating BN_CTX" << std::endl;
      print_openssl_error ();
      EC_KEY_free (ec_key);
      return {};
    }

  char *pub_key_hex = EC_POINT_point2hex (EC_KEY_get0_group (ec_key), pub_key_point,
                                          POINT_CONVERSION_COMPRESSED, ctx);
  if (!pub_key_hex)
    {
      std::cerr << "Error converting public key to hex" << std::endl;
      print_openssl_error ();
      BN_CTX_free (ctx);
      EC_KEY_free (ec_key);
      return {};
    }

  // Remove prefix from the PK
  std::string pub_key_hex_str (pub_key_hex);
  std::string prefix = pub_key_hex_str.substr (0, 2);
  if (prefix == "02")
    prefix = "compressed_y_0";
  else if (prefix == "03")
    prefix = "compressed_y_1";

  pub_key_hex_str = pub_key_hex_str.substr (2);

  publicKey.prefix = prefix;
  publicKey.pk = pub_key_hex_str;

  EC_KEY_free(ec_key);

  return publicKey;
}

// Function to sign a hash with a private key
ECDSA_SIG *
Security::signHash (const unsigned char *hash, EC_KEY *ec_key)
{
  ECDSA_SIG *signature = ECDSA_do_sign (hash, SHA256_DIGEST_LENGTH, ec_key);
  if (!signature)
    {
      std::cerr << "Error signing hash" << std::endl;
      print_openssl_error ();
    }
  return signature;
}

Security::GNsignMaterial
Security::signatureCreation (const std::string& tbsData_hex, const std::string& certificate_hex)
{

  GNsignMaterial signMaterial;

  std::vector<unsigned char> tbsData_bytes = hexStringToBytes (tbsData_hex);
  std::vector<unsigned char> certificate_bytes = hexStringToBytes (certificate_hex);

  unsigned char tbsData_hash[SHA256_DIGEST_LENGTH];
  computeSHA256 (tbsData_bytes, tbsData_hash);

  unsigned char certificate_hash[SHA256_DIGEST_LENGTH];
  computeSHA256 (certificate_bytes, certificate_hash);

  std::vector<unsigned char> concatenatedHashes =
      concatenateHashes (tbsData_hash, certificate_hash);

  unsigned char final_hash[SHA256_DIGEST_LENGTH];
  computeSHA256 (concatenatedHashes, final_hash);

  EC_KEY *ec_key = EC_KEY_dup (m_ecKey);

  // Sign the final hash
  ECDSA_SIG *signature = signHash (final_hash, ec_key);
  if (!signature)
    {
      EC_KEY_free (ec_key);
    }

  // Extract r and s from the signature
  const BIGNUM *r;
  const BIGNUM *s;
  ECDSA_SIG_get0 (signature, &r, &s);


  // Convert r and s to hex strings
  char *r_hex = BN_bn2hex (r);
  char *s_hex = BN_bn2hex (s);

  if (!r_hex || !s_hex) {
      std::cerr << "Error: Failed to convert r or s to hexadecimal." << std::endl;
      ECDSA_SIG_free(signature);
      EC_KEY_free(ec_key);
      return {};
    }


  auto pad_hex_string = [](const char* hex_str) -> std::string {
    std::string padded_hex(hex_str);
    if (padded_hex.length() < 64) {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(64) << padded_hex;
        padded_hex = ss.str();
      }
    return padded_hex;
  };


  std::string r_padded_hex = pad_hex_string(r_hex);
  std::string s_padded_hex = pad_hex_string(s_hex);

  signMaterial.r = r_padded_hex;
  signMaterial.s = s_padded_hex;

  // Clean up
  OPENSSL_free (r_hex);
  OPENSSL_free (s_hex);
  ECDSA_SIG_free (signature);
  EC_KEY_free (ec_key);

  return signMaterial;
}

bool
Security::signatureVerification (const std::string& tbsData_hex, const std::string& certificate_hex, const GNsgtrDC& signatureRS,const std::string& verifyKeyIndicator)
{

  // Convert hex string to bytes
  std::vector<unsigned char> tbsData_bytes = hexStringToBytes (tbsData_hex);
  std::vector<unsigned char> certificate_bytes = hexStringToBytes (certificate_hex);

  // Compute SHA-256 hash
  unsigned char tbsData_hash[SHA256_DIGEST_LENGTH];
  computeSHA256 (tbsData_bytes, tbsData_hash);

  unsigned char certificate_hash[SHA256_DIGEST_LENGTH];
  computeSHA256 (certificate_bytes, certificate_hash);

  // Concatenate the hashes
  std::vector<unsigned char> concatenatedHashes =
      concatenateHashes (tbsData_hash, certificate_hash);

  // Compute SHA-256 hash of the concatenated hashes
  unsigned char final_hash[SHA256_DIGEST_LENGTH];
  computeSHA256 (concatenatedHashes, final_hash);

  std::string r_hex;
  // Here put the hexadecimal string related rSig, sSig (inside outer signature in CAM) and pk (inside verifyKeyIndicator in certificate)
  if (!signatureRS.rSig.p256_x_only.empty()) {
      r_hex = signatureRS.rSig.p256_x_only;
    } else if (!signatureRS.rSig.p256_compressed_y_0.empty()) {
      r_hex = signatureRS.rSig.p256_compressed_y_0;
    } else if (!signatureRS.rSig.p256_compressed_y_1.empty()) {
      r_hex = signatureRS.rSig.p256_compressed_y_1;
    }
  std::string s_hex = signatureRS.sSig;

  // Convert hex strings to bytes

  std::vector<unsigned char> r_bytes(r_hex.begin(), r_hex.end());
  std::vector<unsigned char> s_bytes(s_hex.begin(), s_hex.end());
  std::vector<unsigned char> pk_bytes(verifyKeyIndicator.begin(), verifyKeyIndicator.end());


  // Ensure pk_bytes size is correct for compressed public key
  if (pk_bytes.size () != 33)
    {
      std::cerr << "Error: Public key size is incorrect, expected 33 bytes for compressed key"
                << std::endl;
    }

  // Create the public key object
  EC_KEY *ec_key = EC_KEY_new_by_curve_name (NID_X9_62_prime256v1);
  if (!ec_key)
    {
      std::cerr << "Error creating EC_KEY object" << std::endl;
      print_openssl_error ();
    }

  EC_POINT *pub_key_point = EC_POINT_new (EC_KEY_get0_group (ec_key));
  if (!pub_key_point)
    {
      std::cerr << "Error creating EC_POINT object" << std::endl;
      print_openssl_error ();
      EC_KEY_free (ec_key);
    }

  // Convert the public key from octet string (compressed form)
  if (!EC_POINT_oct2point (EC_KEY_get0_group (ec_key), pub_key_point, pk_bytes.data (),
                           pk_bytes.size (), nullptr))
    {
      std::cerr << "Error converting public key" << std::endl;
      print_openssl_error ();
      EC_POINT_free (pub_key_point);
      EC_KEY_free (ec_key);
    }

  // Set the public key
  if (!EC_KEY_set_public_key (ec_key, pub_key_point))
    {
      std::cerr << "Error setting public key" << std::endl;
      print_openssl_error ();
      EC_POINT_free (pub_key_point);
      EC_KEY_free (ec_key);
    }

  // Create the ECDSA_SIG object
  ECDSA_SIG *signature = ECDSA_SIG_new ();
  if (!signature)
    {
      std::cerr << "Error creating ECDSA_SIG object" << std::endl;
      print_openssl_error ();
      EC_POINT_free (pub_key_point);
      EC_KEY_free (ec_key);
    }

  // Convert r and s in bignum objects
  BIGNUM *r = BN_bin2bn (r_bytes.data (), r_bytes.size (), nullptr);
  BIGNUM *s = BN_bin2bn (s_bytes.data (), s_bytes.size (), nullptr);

  if (!r || !s)
    {
      std::cerr << "Error converting r or s" << std::endl;
      print_openssl_error ();
      ECDSA_SIG_free (signature);
      EC_POINT_free (pub_key_point);
      EC_KEY_free (ec_key);
      if (r)
        BN_free (r);
      if (s)
        BN_free (s);
    }

  // Setting signature through r and s values
  if (!ECDSA_SIG_set0 (signature, r, s))
    {
      std::cerr << "Error setting r and s in signature" << std::endl;
      print_openssl_error ();
      ECDSA_SIG_free (signature);
      EC_POINT_free (pub_key_point);
      EC_KEY_free (ec_key);
    }

  // Verify the signature
  int verify_status = ECDSA_do_verify (final_hash, SHA256_DIGEST_LENGTH, signature, ec_key);
  if (verify_status == 1)
    {
      NS_LOG_INFO ("Signature is valid, received_hash == computed_hash");
      validSignature = true;
    }
  else if (verify_status == 0)
    {
      NS_LOG_INFO ("Signature is invalid");
      validSignature = false;
    }
  else
    {
      std::cerr << "Error verifying signature" << std::endl;
      print_openssl_error ();
    }

  // Clean up
  ECDSA_SIG_free (signature);
  EC_POINT_free (pub_key_point);
  EC_KEY_free (ec_key);

  return validSignature;
}

GNDataRequest_t
Security::createSecurePacket (GNDataRequest_t dataRequest)
{

  auto ieeeData = asn1cpp::makeSeq (Ieee1609Dot2Data);

  // IeeeData, protocol version
  asn1cpp::setField (ieeeData->protocolVersion, m_protocolVersion);

  // IeeeContent
  auto ieeeContent = asn1cpp::makeSeq (Ieee1609Dot2Content);
  asn1cpp::setField (ieeeContent->present, Ieee1609Dot2Content_PR_signedData);

  // SignedData part
  auto signData = asn1cpp::makeSeq (SignedData);

  // HashID field
  asn1cpp::setField (signData->hashId, m_hashId);

  // TobeSigned field
  auto tbs = asn1cpp::makeSeq (ToBeSignedData);
  auto signPayload = asn1cpp::makeSeq (SignedDataPayload);
  auto dataPayload = asn1cpp::makeSeq (Ieee1609Dot2Data);
  asn1cpp::setField (dataPayload->protocolVersion, m_protocolVersion);
  auto dataContentPayload = asn1cpp::makeSeq (Ieee1609Dot2Content);
  asn1cpp::setField (dataContentPayload->present, Ieee1609Dot2Content_PR_unsecuredData);

  uint8_t *buffer; //= new uint8_t[packet->GetSize ()];
  buffer = (uint8_t *) malloc ((dataRequest.data->GetSize ()) * sizeof (uint8_t));
  dataRequest.data->CopyData (buffer, dataRequest.data->GetSize ());
  std::string packetContent ((char *) buffer, (int) dataRequest.data->GetSize ());

  asn1cpp::setField (
      dataContentPayload->choice.unsecuredData,
      packetContent); //205002800093010014007e81ff81274889e4482d1adc36560490ca3f821600800000a000
  asn1cpp::setField (dataPayload->content, dataContentPayload);
  asn1cpp::setField (signPayload->data, dataPayload);
  asn1cpp::setField (tbs->payload, signPayload);

  asn1cpp::setField (tbs->headerInfo.psid, m_psid);
  asn1cpp::setField (
      tbs->headerInfo.generationTime,
      m_generationTime); //generationTime: 2023-12-11 18:45:16.939330 (629405121939330)
  asn1cpp::setField (signData->tbsData, tbs);

  // For each second it will send a signer part with certificate, otherwise it will send digest.

  if (Simulator::Now ().GetMilliSeconds () - m_timestampLastCertificate >= 1000 ||
      m_timestampLastCertificate == 0)
    {

      m_timestampLastCertificate = Simulator::Now ().GetMilliSeconds ();

      GNpublicKey public_key = generateECKeyPair ();

      asn1cpp::setField (signData->signer.present, SignerIdentifier_PR_certificate);
      // Signed Data, Signer part with always 1 Certificate in the Sequence.
      auto certList = asn1cpp::makeSeq (SequenceOfCertificate);
      auto certificate = asn1cpp::makeSeq (CertificateBase);
      asn1cpp::setField (certificate->version, m_version);
      asn1cpp::setField (certificate->type, CertificateType_explicit);
      asn1cpp::setField (certificate->issuer.present, IssuerIdentifierSec_PR_sha256AndDigest);
      asn1cpp::setField (certificate->issuer.choice.sha256AndDigest, m_issuer);
      asn1cpp::setField (certificate->toBeSigned.id.present, CertificateId_PR_none);
      asn1cpp::setField (certificate->toBeSigned.id.choice.none, m_id);
      asn1cpp::setField (certificate->toBeSigned.cracaId, m_cracaId);
      asn1cpp::setField (certificate->toBeSigned.crlSeries, m_crlSeries);
      asn1cpp::setField (certificate->toBeSigned.validityPeriod.start,
                         m_validityPeriod_start); // start: 2023-12-05 03:00:00 (628830005)
      asn1cpp::setField (certificate->toBeSigned.validityPeriod.duration.present,
                         Duration_PR_hours);
      asn1cpp::setField (certificate->toBeSigned.validityPeriod.duration.choice.hours, m_validityPeriod_duration);

      auto appPermission = asn1cpp::makeSeq (SequenceOfPsidSsp);

      // Always two items in SequenceofPsisSsp, App permission
      auto psid1 = asn1cpp::makeSeq (PsidSsp);
      asn1cpp::setField (psid1->psid, m_psid);
      auto servicePermission1 = asn1cpp::makeSeq (ServiceSpecificPermissions);
      asn1cpp::setField (servicePermission1->present, ServiceSpecificPermissions_PR_bitmapSsp);
      asn1cpp::setField (servicePermission1->choice.bitmapSsp, m_bitmapSsp);
      asn1cpp::setField (psid1->ssp, servicePermission1);
      asn1cpp::sequenceof::pushList (*appPermission, psid1);

      auto psid2 = asn1cpp::makeSeq (PsidSsp);
      asn1cpp::setField (psid2->psid, m_psid2);
      auto servicePermission2 = asn1cpp::makeSeq (ServiceSpecificPermissions);
      asn1cpp::setField (servicePermission2->present, ServiceSpecificPermissions_PR_bitmapSsp);
      asn1cpp::setField (servicePermission2->choice.bitmapSsp, m_bitmapSsp2);
      asn1cpp::setField (psid2->ssp, servicePermission2);
      asn1cpp::sequenceof::pushList (*appPermission, psid2);
      asn1cpp::setField (certificate->toBeSigned.appPermissions, appPermission);


      asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.present,
                         VerificationKeyIndicator_PR_verificationKey);
      asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.present,
                         PublicVerificationKey_PR_ecdsaNistP256);
      if (public_key.prefix == "compressed_y_0")
        {
          asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                 .choice.ecdsaNistP256.present,
                             EccP256CurvePoint_PR_compressed_y_0);
          std::vector<unsigned char> pk_bytes = hexStringToBytes (public_key.pk);
          std::string pk_string(pk_bytes.begin(), pk_bytes.end());
          asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                 .choice.ecdsaNistP256.choice.compressed_y_0,
                             pk_string);
        }
      else if (public_key.prefix == "compressed_y_1")
        {
          asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                 .choice.ecdsaNistP256.present,
                             EccP256CurvePoint_PR_compressed_y_1);
          std::vector<unsigned char> pk_bytes = hexStringToBytes (public_key.pk);
          std::string pk_string(pk_bytes.begin(), pk_bytes.end());
          asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                 .choice.ecdsaNistP256.choice.compressed_y_1,
                             pk_string);
        }
      auto signatureCert = asn1cpp::makeSeq (Signature);
      asn1cpp::setField (signatureCert->present, Signature_PR_ecdsaNistP256Signature);

      asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.present,
                       EccP256CurvePoint_PR_x_only);
      asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.choice.x_only,
                       m_p256_x_only_Cert);
      asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.sSig,
                         m_SsigCert);
      asn1cpp::setField (certificate->signature, signatureCert);

      asn1cpp::sequenceof::pushList (*certList, certificate);
      asn1cpp::setField (signData->signer.choice.certificate, certList);

      std::string certHex = asn1cpp::oer::encode (certificate);
      m_certificate = certHex;
    }
  else
    {

      // Signed Data, Signer field: version with only digest, other wireshark message with certificate option
      asn1cpp::setField (signData->signer.present, SignerIdentifier_PR_digest);
      asn1cpp::setField (signData->signer.choice.digest, m_digest);
    }

  std::string tbs_hex = asn1cpp::oer::encode (tbs);
  GNsignMaterial sign_material = signatureCreation (tbs_hex, m_certificate);

  // Signed Data, Signature part,
  auto signatureContent = asn1cpp::makeSeq (Signature);
  asn1cpp::setField (signatureContent->present, Signature_PR_ecdsaNistP256Signature);
  asn1cpp::setField (signatureContent->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_x_only);
  std::vector<unsigned char> R_bytes = hexStringToBytes (sign_material.r);
  std::string r_string(R_bytes.begin(), R_bytes.end());
  asn1cpp::setField (signatureContent->choice.ecdsaNistP256Signature.rSig.choice.x_only, r_string);
  std::vector<unsigned char> S_bytes = hexStringToBytes (sign_material.s);
  std::string s_string(S_bytes.begin(), S_bytes.end());
  asn1cpp::setField (signatureContent->choice.ecdsaNistP256Signature.sSig, s_string);
  asn1cpp::setField (signData->signature, signatureContent);

  asn1cpp::setField (ieeeContent->choice.signedData, signData);
  asn1cpp::setField (ieeeData->content, ieeeContent);

  // data encode
  std::string encode_result = asn1cpp::oer::encode (ieeeData);

  if(encode_result.empty()){
      std::cout << "Error encoding data" << std::endl;
      NS_LOG_ERROR ("Error encoding data");
    }

  Ptr<Packet> packet = Create<Packet> ((uint8_t *) encode_result.c_str (), encode_result.size ());
  dataRequest.data = packet;
  free (buffer);
  return dataRequest;
}

Security::Security_error_t
Security::extractSecurePacket (GNDataIndication_t &dataIndication)
{

  asn1cpp::Seq<Ieee1609Dot2Data> ieeeData_decoded;
  uint8_t *buffer; //= new uint8_t[packet->GetSize ()];
  buffer = (uint8_t *) malloc ((dataIndication.data->GetSize ()) * sizeof (uint8_t));
  dataIndication.data->CopyData (buffer, dataIndication.data->GetSize ());
  std::string packetContent ((char *) buffer, (int) dataIndication.data->GetSize ());
  ieeeData_decoded = asn1cpp::oer::decodeASN (packetContent, Ieee1609Dot2Data);
  free (buffer);

  GNsecDP secureDataPacket;

  NS_LOG_INFO ("-----------------------------------------");
  NS_LOG_INFO ("        Values secure packet decoded     ");
  NS_LOG_INFO ("-----------------------------------------");

  secureDataPacket.protocol_version = asn1cpp::getField (ieeeData_decoded->protocolVersion, long);
  NS_LOG_INFO (
      "Ieee1609Dot2Data container, protocol version: " << secureDataPacket.protocol_version);
  // boolean value for getSeq, getSeqOpt
  bool getValue_ok;

  // content of Ieee1609Dot2Data
  auto contentDecoded =
      asn1cpp::getSeqOpt (ieeeData_decoded->content, Ieee1609Dot2Content, &getValue_ok);

  // check the present, here is always signed data
  auto present1 = asn1cpp::getField (contentDecoded->present, Ieee1609Dot2Content_PR);
  if (present1 == Ieee1609Dot2Content_PR_signedData)
    {
      auto signedDataDecoded =
          asn1cpp::getSeqOpt (contentDecoded->choice.signedData, SignedData, &getValue_ok);

      // First signed data field, HASH ID
      secureDataPacket.content.signData.hashId =
          asn1cpp::getField (signedDataDecoded->hashId, long);
      NS_LOG_INFO ("SignedData container, HASH ID: " << secureDataPacket.content.signData.hashId);

      // Second signed data field, TBSDATA, inside there is another Ieee1609Dot2Data container with unsercuredData present.
      auto tbsDecoded =
          asn1cpp::getSeqOpt (signedDataDecoded->tbsData, ToBeSignedData, &getValue_ok);
      auto payload_decoded =
          asn1cpp::getSeqOpt (tbsDecoded->payload, SignedDataPayload, &getValue_ok);
      auto dataContainerDecoded =
          asn1cpp::getSeqOpt (payload_decoded->data, Ieee1609Dot2Data, &getValue_ok);
      secureDataPacket.content.signData.tbsData.protocol_version =
          asn1cpp::getField (dataContainerDecoded->protocolVersion, long);
      NS_LOG_INFO (
          "SignedData outer container,TBSDATA, Ieee1609Dot2Data inner container, protocol version: "
          << secureDataPacket.content.signData.tbsData.protocol_version);
      auto contentContainerDecoded =
          asn1cpp::getSeqOpt (dataContainerDecoded->content, Ieee1609Dot2Content);
      auto present2 = asn1cpp::getField (contentContainerDecoded->present, Ieee1609Dot2Content_PR);
      if (present2 == Ieee1609Dot2Content_PR_unsecuredData)
        {
          secureDataPacket.content.signData.tbsData.unsecureData =
              asn1cpp::getField (contentContainerDecoded->choice.unsecuredData, std::string);

          NS_LOG_INFO ("SignedData outer container,TBSDATA, Ieee1609Dot2Content inner container, "
                       "unsecuredData: "
                       << secureDataPacket.content.signData.tbsData.unsecureData);
        }
      // else if( present2 == ??) Is it needed? Never present
      secureDataPacket.content.signData.tbsData.headerInfo_psid =
          asn1cpp::getField (tbsDecoded->headerInfo.psid, unsigned long);
      NS_LOG_INFO ("SignedData container, TBSDATA, headerinfo PSID: "
                   << secureDataPacket.content.signData.tbsData.headerInfo_psid);
      secureDataPacket.content.signData.tbsData.headerInfo_generetionTime =
          asn1cpp::getField (tbsDecoded->headerInfo.generationTime, uint64_t);
      NS_LOG_INFO ("SignedData container, TBSDATA, headerinfo GENERATION TIME: "
                   << secureDataPacket.content.signData.tbsData.headerInfo_generetionTime);

      // Third signed data field, SIGNER, can be "digest" or "certificate"
      auto present3 = asn1cpp::getField (signedDataDecoded->signer.present, SignerIdentifier_PR);

      if (present3 == SignerIdentifier_PR_digest)
        {
          secureDataPacket.content.signData.signerId.digest =
              asn1cpp::getField (signedDataDecoded->signer.choice.digest, std::string);
          NS_LOG_INFO ("SignedData container, SIGNER, DIGEST: "
                       << secureDataPacket.content.signData.signerId.digest);
        }
      else if (present3 == SignerIdentifier_PR_certificate)
        {
          std::string verificationKey;
          //There is always only one certificate, but to be sure a for is implemented.
          int size = asn1cpp::sequenceof::getSize (signedDataDecoded->signer.choice.certificate);
          for (int i = 0; i < size; i++)
            {
              // Filling all certificate fields
              auto certDecoded = asn1cpp::sequenceof::getSeq (
                  signedDataDecoded->signer.choice.certificate, CertificateBase, i);
              GNcertificateDC newCert;
              newCert.version = asn1cpp::getField (certDecoded->version, long);
              NS_LOG_INFO (
                  "SignedData container, SIGNER, CERTIFICATE VERSION: " << newCert.version);
              newCert.type = asn1cpp::getField (certDecoded->type, long);
              NS_LOG_INFO ("SignedData container, SIGNER, CERTIFICATE TYPE: " << newCert.type);

              if (asn1cpp::getField (certDecoded->issuer.present, IssuerIdentifierSec_PR) ==
                  IssuerIdentifierSec_PR_sha256AndDigest)
                {
                  newCert.issuer = asn1cpp::getField (
                      certDecoded->issuer.choice.sha256AndDigest, std::string);
                  NS_LOG_INFO (
                      "SignedData container, SIGNER, CERTIFICATE ISSUER: " << newCert.issuer);
                }

              if (asn1cpp::getField (certDecoded->toBeSigned.id.present, CertificateId_PR) ==
                  CertificateId_PR_none)
                {
                  //getField(certDecoded->toBeSigned.id.choice.none, long ); all types give error, so there is a string. Value is meaningless.
                  newCert.tbs.id = 0;
                  NS_LOG_INFO (
                      "SignedData container, SIGNER, CERTIFICATE TBS ID: " << newCert.tbs.id);
                }
              // else if (certDecoded->toBeSigned.id.present == name or binary or linkageData). Never present
              newCert.tbs.cracaId =
                  asn1cpp::getField (certDecoded->toBeSigned.cracaId, std::string);
              NS_LOG_INFO (
                  "SignedData container, SIGNER, CERTIFICATE TBS CRACAID: " << newCert.tbs.cracaId);
              newCert.tbs.crlSeries =
                  asn1cpp::getField (certDecoded->toBeSigned.crlSeries, uint16_t);
              NS_LOG_INFO ("SignedData container, SIGNER, CERTIFICATE TBS CRLSERIES: "
                           << newCert.tbs.crlSeries);
              newCert.tbs.validityPeriod_start =
                  asn1cpp::getField (certDecoded->toBeSigned.validityPeriod.start, uint32_t);
              NS_LOG_INFO ("SignedData container, SIGNER, CERTIFICATE TBS VALIDITY PERIOD START: "
                           << newCert.tbs.validityPeriod_start);
              if (asn1cpp::getField (certDecoded->toBeSigned.validityPeriod.duration.present,
                                     Duration_PR) == Duration_PR_hours)
                {
                  newCert.tbs.validityPeriod_duration = asn1cpp::getField (
                      certDecoded->toBeSigned.validityPeriod.duration.choice.hours, long);
                  NS_LOG_INFO (
                      "SignedData container, SIGNER, CERTIFICATE TBS VALIDITY PERIOD DURATION: "
                      << newCert.tbs.validityPeriod_duration);
                }
              // else if (certDecoded->toBeSigned.validityPeriod.duration.present == minutes, seconds, etc...). Never present
              // Manage the list of APP PERMISSIONS. There will be always two items, but to be sure a for is implemented.

              int size2 = asn1cpp::sequenceof::getSize (certDecoded->toBeSigned.appPermissions);
              for (int j = 0; j < size2; j++)
                {
                  auto appPermDecoded = asn1cpp::sequenceof::getSeq (
                      certDecoded->toBeSigned.appPermissions, PsidSsp, j, &getValue_ok);
                  GNpsidSsp newServ;
                  newServ.psid = asn1cpp::getField (appPermDecoded->psid, unsigned long);
                  NS_LOG_INFO ("SignedData container, SIGNER, CERTIFICATE TBS APP PERMISSION PSID: "
                               << newServ.psid);
                  auto servicePermission = asn1cpp::getSeqOpt (
                      appPermDecoded->ssp, ServiceSpecificPermissions, &getValue_ok);
                  if (asn1cpp::getField (servicePermission->present,
                                         ServiceSpecificPermissions_PR) ==
                      ServiceSpecificPermissions_PR_bitmapSsp)
                    {
                      newServ.bitmapSsp =
                          asn1cpp::getField (servicePermission->choice.bitmapSsp, std::string);
                      NS_LOG_INFO (
                          "SignedData container, SIGNER, CERTIFICATE TBS APP PERMISSION BITMAPSSP: "
                          << newServ.bitmapSsp);
                    } // else if( servicePermission->present == opaque or null). Never present
                  newCert.tbs.appPermissions.push_back (newServ);
                }
              // Filling last certificate field, verifyKeyIndicator.
              if (asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.present,
                                     VerificationKeyIndicator_PR) ==
                  VerificationKeyIndicator_PR_verificationKey)
                {
                  if (asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice
                                             .verificationKey.present,
                                         PublicVerificationKey_PR) ==
                      PublicVerificationKey_PR_ecdsaNistP256)
                    {

                      std::vector<unsigned char> prefix_y_0 = hexStringToBytes ("02");
                      std::string y0_string(prefix_y_0.begin(), prefix_y_0.end());
                      std::vector<unsigned char> prefix_y_1 = hexStringToBytes ("03");
                      std::string y1_string(prefix_y_1.begin(), prefix_y_1.end());

                      switch (
                          asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice
                                                 .verificationKey.choice.ecdsaNistP256.present,
                                             EccP256CurvePoint_PR))
                        {
                        case EccP256CurvePoint_PR_x_only:
                          newCert.tbs.verifyKeyIndicator.p256_x_only = asn1cpp::getField (
                              certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                  .choice.ecdsaNistP256.choice.x_only,
                              std::string);
                          NS_LOG_INFO (
                              "CERTIFICATE verifyKeyIndicator, EccP256CurvePoint_PR_x_only: "
                              << newCert.tbs.verifyKeyIndicator.p256_x_only);
                          break;
                        case EccP256CurvePoint_PR_fill:
                          newCert.tbs.verifyKeyIndicator.p256_fill = "NULL";
                          NS_LOG_INFO (
                              "CERTIFICATE verifyKeyIndicator, EccP256CurvePoint_PR_fill value is: "
                              << newCert.tbs.verifyKeyIndicator.p256_fill);
                          break;
                        case EccP256CurvePoint_PR_compressed_y_0:
                          newCert.tbs.verifyKeyIndicator.p256_compressed_y_0 = asn1cpp::getField (
                              certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                  .choice.ecdsaNistP256.choice.compressed_y_0,
                              std::string);
                          verificationKey = y0_string + newCert.tbs.verifyKeyIndicator.p256_compressed_y_0;
                          NS_LOG_INFO ("CERTIFICATE verifyKeyIndicator, "
                                       "EccP256CurvePoint_PR_compressed_y_0: "
                                       << newCert.tbs.verifyKeyIndicator.p256_compressed_y_0);
                          break;
                        case EccP256CurvePoint_PR_compressed_y_1:
                          newCert.tbs.verifyKeyIndicator.p256_compressed_y_1 = asn1cpp::getField (
                              certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                  .choice.ecdsaNistP256.choice.compressed_y_1,
                              std::string);
                          verificationKey = y1_string + newCert.tbs.verifyKeyIndicator.p256_compressed_y_1;
                          NS_LOG_INFO ("CERTIFICATE verifyKeyIndicator, "
                                       "EccP256CurvePoint_PR_compressed_y_1: "
                                       << newCert.tbs.verifyKeyIndicator.p256_compressed_y_1);
                          break;
                        case EccP256CurvePoint_PR_uncompressedP256:
                          newCert.tbs.verifyKeyIndicator.p256_uncompressed_x = asn1cpp::getField (
                              certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                  .choice.ecdsaNistP256.choice.uncompressedP256.x,
                              std::string);
                          NS_LOG_INFO ("CERTIFICATE verifyKeyIndicator, "
                                       "EccP256CurvePoint_PR_uncompressedP256_x: "
                                       << newCert.tbs.verifyKeyIndicator.p256_uncompressed_x);
                          newCert.tbs.verifyKeyIndicator.p256_uncompressed_y = asn1cpp::getField (
                              certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey
                                  .choice.ecdsaNistP256.choice.uncompressedP256.y,
                              std::string);
                          NS_LOG_INFO ("CERTIFICATE verifyKeyIndicator, "
                                       "EccP256CurvePoint_PR_uncompressedP256_y: "
                                       << newCert.tbs.verifyKeyIndicator.p256_uncompressed_y);
                          break;
                        default:
                          break;
                        }

                    } // else if(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.present == ecc..). Never present

                } // else if (certDecoded->toBeSigned.verifyKeyIndicator.present == VerificationKeyIndicator_PR_reconstructionValue). Never present

              // Signature part inside certificate signer
              auto signCertDecoded =
                  asn1cpp::getSeqOpt (certDecoded->signature, Signature, &getValue_ok);
              if (asn1cpp::getField (signCertDecoded->present, Signature_PR) ==
                  Signature_PR_ecdsaNistP256Signature)
                {
                  auto present4 = asn1cpp::getField (
                      signCertDecoded->choice.ecdsaNistP256Signature.rSig.present,
                      EccP256CurvePoint_PR);
                  switch (present4)
                    {
                    case EccP256CurvePoint_PR_x_only:
                      newCert.signature.rSig.p256_x_only = asn1cpp::getField (
                          signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.x_only,
                          std::string);
                      NS_LOG_INFO ("CERTIFICATE SIGNATURE, rSig EccP256CurvePoint_PR_x_only: "
                                   << newCert.signature.rSig.p256_x_only);
                      break;
                    case EccP256CurvePoint_PR_fill:
                      newCert.signature.rSig.p256_fill = "NULL";
                      NS_LOG_INFO (
                          "CERTIFICATE SIGNATURE, rSig EccP256CurvePoint_PR_fill value is: "
                          << newCert.signature.rSig.p256_fill);
                      break;
                    case EccP256CurvePoint_PR_compressed_y_0:
                      newCert.signature.rSig.p256_compressed_y_0 = asn1cpp::getField (
                          signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_0,
                          std::string);
                      NS_LOG_INFO (
                          "CERTIFICATE SIGNATURE, rSig EccP256CurvePoint_PR_compressed_y_0: "
                          << newCert.signature.rSig.p256_compressed_y_0);
                      break;
                    case EccP256CurvePoint_PR_compressed_y_1:
                      newCert.signature.rSig.p256_compressed_y_1 = asn1cpp::getField (
                          signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_1,
                          std::string);
                      NS_LOG_INFO (
                          "CERTIFICATE SIGNATURE, rSig EccP256CurvePoint_PR_compressed_y_1: "
                          << newCert.signature.rSig.p256_compressed_y_1);
                      break;
                    case EccP256CurvePoint_PR_uncompressedP256:
                      newCert.signature.rSig.p256_uncompressed_x =
                          asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig
                                                 .choice.uncompressedP256.x,
                                             std::string);
                      NS_LOG_INFO (
                          "CERTIFICATE SIGNATURE, rSig EccP256CurvePoint_PR_uncompressedP256_x: "
                          << newCert.signature.rSig.p256_uncompressed_x);
                      newCert.signature.rSig.p256_uncompressed_y =
                          asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig
                                                 .choice.uncompressedP256.y,
                                             std::string);
                      NS_LOG_INFO (
                          "CERTIFICATE SIGNATURE, rSig EccP256CurvePoint_PR_uncompressedP256_y: "
                          << newCert.signature.rSig.p256_uncompressed_y);
                      break;
                    default:
                      break;
                    }
                  newCert.signature.sSig = asn1cpp::getField (
                      signCertDecoded->choice.ecdsaNistP256Signature.sSig, std::string);
                  NS_LOG_INFO ("CERTIFICATE SIGNATURE, ecdsaNistP256Signature sSig: "
                               << newCert.signature.sSig);
                } // else if(signCertDecoded->present == etc...). Never present
              secureDataPacket.content.signData.signerId.certificate.push_back (newCert);

              std::string certHex = asn1cpp::oer::encode (certDecoded);
              std::pair<std::string, std::string> pair = std::make_pair (verificationKey,certHex);
              uint64_t timestamp = Simulator::Now ().GetMilliSeconds ();
              m_receivedCertificates[timestamp] = pair;
            }
        }

      //Signature part of signed data
      if (asn1cpp::getField (signedDataDecoded->signature.present, Signature_PR) ==
          Signature_PR_ecdsaNistP256Signature)
        {

          auto present5 = asn1cpp::getField (
              signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.present,
              EccP256CurvePoint_PR);
          switch (present5)
            {
            case EccP256CurvePoint_PR_x_only:
              secureDataPacket.content.signData.signature.rSig.p256_x_only = asn1cpp::getField (
                  signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.choice.x_only,
                  std::string);
              NS_LOG_INFO ("SignedData container, SIGNATURE, rSig EccP256CurvePoint_PR_x_only: "
                           << secureDataPacket.content.signData.signature.rSig.p256_x_only);
              break;
            case EccP256CurvePoint_PR_fill:
              secureDataPacket.content.signData.signature.rSig.p256_fill = "NULL";
              NS_LOG_INFO (
                  "SignedData container, SIGNATURE, rSig EccP256CurvePoint_PR_fill value is: "
                  << secureDataPacket.content.signData.signature.rSig.p256_fill);
              break;
            case EccP256CurvePoint_PR_compressed_y_0:
              secureDataPacket.content.signData.signature.rSig.p256_compressed_y_0 =
                  asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig
                                         .choice.compressed_y_0,
                                     std::string);
              NS_LOG_INFO (
                  "SignedData container, SIGNATURE, rSig EccP256CurvePoint_PR_compressed_y_0: "
                  << secureDataPacket.content.signData.signature.rSig.p256_compressed_y_0);
              break;
            case EccP256CurvePoint_PR_compressed_y_1:
              secureDataPacket.content.signData.signature.rSig.p256_compressed_y_1 =
                  asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig
                                         .choice.compressed_y_1,
                                     std::string);
              NS_LOG_INFO (
                  "SignedData container, SIGNATURE, rSig EccP256CurvePoint_PR_compressed_y_1: "
                  << secureDataPacket.content.signData.signature.rSig.p256_compressed_y_1);
              break;
            case EccP256CurvePoint_PR_uncompressedP256:
              secureDataPacket.content.signData.signature.rSig.p256_uncompressed_x =
                  asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig
                                         .choice.uncompressedP256.x,
                                     std::string);
              NS_LOG_INFO (
                  "SignedData container, SIGNATURE, rSig EccP256CurvePoint_PR_uncompressedP256_X: "
                  << secureDataPacket.content.signData.signature.rSig.p256_uncompressed_x);
              secureDataPacket.content.signData.signature.rSig.p256_uncompressed_y =
                  asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig
                                         .choice.uncompressedP256.y,
                                     std::string);
              NS_LOG_INFO (
                  "SignedData container, SIGNATURE, rSig EccP256CurvePoint_PR_uncompressedP256_y: "
                  << secureDataPacket.content.signData.signature.rSig.p256_uncompressed_y);
              break;
            default:
              break;
            }
          secureDataPacket.content.signData.signature.sSig = asn1cpp::getField (
              signedDataDecoded->signature.choice.ecdsaNistP256Signature.sSig, std::string);
          NS_LOG_INFO ("SignedData container, SIGNATURE, ecdsaNistP256Signature Ssig: "
                       << secureDataPacket.content.signData.signature.sSig);
        }

      std::string tbs_hex = asn1cpp::oer::encode (tbsDecoded);
      if (m_receivedCertificates.empty()){
          NS_LOG_INFO("No certificate received");
          return SECURITY_VERIFICATION_FAILED;
      }else {
          //for every item in map do signature verification
          bool signValid = false;
          for (auto const &item : m_receivedCertificates) {
              if (signatureVerification(tbs_hex, item.second.second,secureDataPacket.content.signData.signature,item.second.first)) {
                  signValid = true;
                  break;
              }
              NS_LOG_ERROR("Signature verification failed for current certificate");
          }
          if (!signValid) {
            return SECURITY_VERIFICATION_FAILED;
          }
      }
    }
  else if (present1 == Ieee1609Dot2Content_PR_unsecuredData)
    { // Is it needed? Never present
      secureDataPacket.content.unsecuredData =
          asn1cpp::getField (contentDecoded->choice.unsecuredData, std::string);
      NS_LOG_INFO ("UnsecuredData value is: " << secureDataPacket.content.unsecuredData);
    }


  Ptr<Packet> packet =
      Create<Packet> ((uint8_t *) secureDataPacket.content.signData.tbsData.unsecureData.c_str (),
                      secureDataPacket.content.signData.tbsData.unsecureData.size ());
  dataIndication.data = packet;
  return SECURITY_OK;
}

} // namespace ns3