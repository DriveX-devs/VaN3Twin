#ifndef SECURITY_H
#define SECURITY_H


#include <string>
#include <openssl/ecdsa.h>
#include <openssl/sha.h>
#include <openssl/err.h>
#include <openssl/bn.h>
#include <openssl/pem.h>
#include "ns3/asn_utils.h"
#include "ns3/btpdatarequest.h"
#include <openssl/obj_mac.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>

extern "C" {
#include "ns3/CAM.h"
}

namespace ns3
{
/**
 * @brief The Security class.
 *
 * This class is responsible for handling the security aspects of the GeoNet protocol.
 * It provides methods to create and extract secure packets.
 */
class Security : public Object
{
public:

  typedef enum {
    SECURITY_OK,
    SECURITY_VERIFICATION_FAILED,
  } Security_error_t;


  // App Permission field
  typedef struct EccP256CurvePoint{
    std::string p256_x_only;
    std::string p256_fill;
    std::string p256_compressed_y_0;
    std::string p256_compressed_y_1;
    std::string p256_uncompressed_x;
    std::string p256_uncompressed_y;
  }GNecdsaNistP256;

  // Service Permission field
  typedef struct _servicePermissions{
    unsigned long psid;
    std::string bitmapSsp;
  }GNpsidSsp;

  // ToBeSignedCertificate Container, Certificate field
  typedef struct _tbsCertificateDataContainer{
    int id; // Actually is a struct with different type, but we consider the case "none" with value 0.
    std::string cracaId;
    uint16_t  crlSeries;
    uint32_t  validityPeriod_start;
    long validityPeriod_duration;
    std::vector<GNpsidSsp> appPermissions;
    GNecdsaNistP256 verifyKeyIndicator; // Ignore other fields beacuse there is always ecdsaNistP256
  }GNtbsCertDC;

  // Signature Container, Signed Data and Certificate field
  typedef struct _signatureDataContainer{
    GNecdsaNistP256 rSig;
    std::string sSig;
  }GNsgtrDC;

  // Certificate list
  typedef struct _certificateDataContainer{
    long version;
    long type;
    std::string issuer;
    GNtbsCertDC tbs;
    GNsgtrDC signature;
  }GNcertificateDC;

  // Signer Container, Signed Data field
  typedef struct _signerIdContainer{
    std::string digest;
    std::vector<GNcertificateDC> certificate;
  }GNsgrC;

  // tbsData Container, Signed Data field
  typedef struct tbsDataContainer{
    long protocol_version;
    std::string unsecureData;
    unsigned long headerInfo_psid;
    uint64_t headerInfo_generetionTime;
  }GNtbsDC;

  // Signed Data Container, Secure Data Packet field
  typedef struct _signedDataContainer{
    long hashId;
    GNtbsDC tbsData;
    GNsgrC signerId;
    GNsgtrDC signature;
  }GNsignDC;

  typedef struct _dataContent{
    std::string unsecuredData;
    GNsignDC signData;
    bool encryptedData; // it is not a bool, for the moment not implemented, never used
    std::string signedCertificateRequest;
    std::string signedX509CertificateRequest;
  }GNcontent;


  // IeeeData Container
  typedef struct _secureDataPacket{
    long protocol_version;
    GNcontent content;
  }GNsecDP;

  typedef struct _publicKey{
    std::string prefix;
    std::string pk;
  }GNpublicKey;

  typedef struct _signatureMaterial{
    std::string r;
    std::string s;
  }GNsignMaterial;

  GNDataRequest_t createSecurePacket(GNDataRequest_t dataRequest);
  Security_error_t extractSecurePacket(GNDataIndication_t &dataIndication);


  static TypeId GetTypeId ();
  /**
   * @brief Construct a new Security object.
   *
   * Default constructor for the Security class.
   */
  Security();
  virtual ~Security();

  void setProtocolVersion(long protocolVersion){m_protocolVersion = protocolVersion;};

  void setHashId(long hashId){m_hashId = hashId;};

  void setPsid(unsigned long psid){m_psid = psid;};

  void setPsid2(unsigned long psid2){m_psid2 = psid2;};

  void setGenerationTime(uint64_t generationTime){m_generationTime = generationTime;};

  void setDigest(std::string digest){m_digest = digest;};

  void setVersion(long version){m_version = version;};

  void setType(long type){m_type = type;};

  void setIssuer(std::string issuer){m_issuer = issuer;};

  void setId(int id){m_id = id;};

  void setCracaId(std::string cracaId){m_cracaId = cracaId;};

  void setCrlSeries(uint16_t crlSeries){m_crlSeries = crlSeries;};

  void setValidityPeriod_start(uint32_t validityPeriod_start){m_validityPeriod_start = validityPeriod_start;};

  void setValidityPeriod_duration(long validityPeriod_duration){m_validityPeriod_duration = validityPeriod_duration;};

  void setBitmapSsp(std::string bitmapSsp){m_bitmapSsp = bitmapSsp;};

  void setBitmapSsp2(std::string bitmapSsp2){m_bitmapSsp2 = bitmapSsp2;};

  void setP256_compressed_x_only(std::string p256_x_only){m_p256_x_only_Cert = p256_x_only;};

  void setSsig(std::string Ssig){m_SsigCert = Ssig;};



private:


  std::vector<unsigned char> hexStringToBytes(const std::string& hex);
  void computeSHA256(const std::vector<unsigned char>& data, unsigned char hash[SHA256_DIGEST_LENGTH]);
  std::vector<unsigned char> concatenateHashes(const unsigned char hash1[SHA256_DIGEST_LENGTH], const unsigned char hash2[SHA256_DIGEST_LENGTH]);
  void print_openssl_error();
  GNpublicKey generateECKeyPair();
  ECDSA_SIG* signHash(const unsigned char* hash, EC_KEY* ec_key);
  GNsignMaterial signatureCreation( const std::string& tbsData_hex,  const std::string& certificate_hex);
  bool signatureVerification( const std::string& tbsData_hex,  const std::string& certificate_hex, const GNsgtrDC& signatureRS, const std::string& verifyKeyIndicator);
  void mapCleaner();

  EventId m_eventCleaner;

  EC_KEY *m_ecKey;
  int64_t m_timestampLastCertificate = 0;
  std::map<uint64_t, std::pair<std::string,std::string>> m_receivedCertificates;
  std::string m_certificate;
  GNpublicKey publicKey;
  bool validSignature;


  // Ieee1609Dot2Data fields

  long m_protocolVersion;
  long m_hashId;
  unsigned long m_psid;
  unsigned long m_psid2;
  uint64_t m_generationTime;
  std::string m_digest;
  long m_version;
  long m_type;
  std::string m_issuer;
  int m_id;
  std::string m_cracaId;
  uint16_t m_crlSeries;
  uint32_t m_validityPeriod_start;
  long m_validityPeriod_duration;
  std::string m_bitmapSsp;
  std::string m_bitmapSsp2;
  std::string m_p256_x_only_Cert;
  std::string m_SsigCert;



};
}
#endif // SECURITY_H
