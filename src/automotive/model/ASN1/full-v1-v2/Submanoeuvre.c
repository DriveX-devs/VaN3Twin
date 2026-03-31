#include "Submanoeuvre.h"

asn_TYPE_member_t asn_MBR_Submanoeuvre_1[] = {
    { ATF_NOFLAGS, 0, offsetof(struct Submanoeuvre, submanoeuvreId),
     (ASN_TAG_CLASS_CONTEXT | (0 << 2)),
     -1,	/* IMPLICIT tag at current level */
     &asn_DEF_Identifier1B,
     0,
     {
#if !defined(ASN_DISABLE_OER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_JER_SUPPORT)
         0,
#endif
         0
     },
     0, 0,
     "submanoeuvreId"
    },
    /* NEW: acceleration at index 1 */
    { ATF_NOFLAGS, 0, offsetof(struct Submanoeuvre, acceleration),
     (ASN_TAG_CLASS_CONTEXT | (1 << 2)),
     -1,	/* IMPLICIT tag at current level */
     &asn_DEF_LongitudinalAcceleration,
     0,
     {
#if !defined(ASN_DISABLE_OER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_JER_SUPPORT)
         0,
#endif
         0
     },
     0, 0,
     "acceleration"
    },
    /* NEW: deltaTime at index 2 */
    { ATF_NOFLAGS, 0, offsetof(struct Submanoeuvre, durationDeltaTime),
     (ASN_TAG_CLASS_CONTEXT | (2 << 2)),
     -1,	/* IMPLICIT tag at current level */
     &asn_DEF_DeltaTimeMilliSecondPositive,
     0,
     {
#if !defined(ASN_DISABLE_OER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_JER_SUPPORT)
         0,
#endif
         0
     },
     0, 0,
     "durationDeltaTime"
    },
    /* SHIFTED: advisedTrajectory now at index 3, tag 3 */
    { ATF_POINTER, 2, offsetof(struct Submanoeuvre, advisedTrajectory),
     (ASN_TAG_CLASS_CONTEXT | (3 << 2)),
     -1,	/* IMPLICIT tag at current level */
     &asn_DEF_Trajectory,
     0,
     {
#if !defined(ASN_DISABLE_OER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_JER_SUPPORT)
         0,
#endif
         0
     },
     0, 0,
     "advisedTrajectory"
    },
    /* SHIFTED: advisedTargetRoadResource now at index 4, tag 4 */
    { ATF_POINTER, 1, offsetof(struct Submanoeuvre, advisedTargetRoadResource),
     (ASN_TAG_CLASS_CONTEXT | (4 << 2)),
     -1,	/* IMPLICIT tag at current level */
     &asn_DEF_AdvisedTrrContainer,
     0,
     {
#if !defined(ASN_DISABLE_OER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
         0,
#endif
#if !defined(ASN_DISABLE_JER_SUPPORT)
         0,
#endif
         0
     },
     0, 0,
     "advisedTargetRoadResource"
    },
};

/* Optional members are now at indices 3 and 4 */
static const int asn_MAP_Submanoeuvre_oms_1[] = { 3, 4 };

static const ber_tlv_tag_t asn_DEF_Submanoeuvre_tags_1[] = {
    (ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};

/* tag2el updated: 5 entries, tags 0-4 */
static const asn_TYPE_tag2member_t asn_MAP_Submanoeuvre_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* submanoeuvreId */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* acceleration */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* deltaTime */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* advisedTrajectory */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }  /* advisedTargetRoadResource */
};

asn_SEQUENCE_specifics_t asn_SPC_Submanoeuvre_specs_1 = {
    sizeof(struct Submanoeuvre),
    offsetof(struct Submanoeuvre, _asn_ctx),
    asn_MAP_Submanoeuvre_tag2el_1,
    5,	/* Count of tags in the map: now 5 */
    asn_MAP_Submanoeuvre_oms_1,	/* Optional members */
    2, 0,	/* Root/Additions: still 2 optional */
    -1,	/* First extension addition */
};

asn_TYPE_descriptor_t asn_DEF_Submanoeuvre = {
    "Submanoeuvre",
    "Submanoeuvre",
    &asn_OP_SEQUENCE,
    asn_DEF_Submanoeuvre_tags_1,
    sizeof(asn_DEF_Submanoeuvre_tags_1)
        /sizeof(asn_DEF_Submanoeuvre_tags_1[0]),
    asn_DEF_Submanoeuvre_tags_1,
    sizeof(asn_DEF_Submanoeuvre_tags_1)
        /sizeof(asn_DEF_Submanoeuvre_tags_1[0]),
    {
#if !defined(ASN_DISABLE_OER_SUPPORT)
        0,
#endif
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
        0,
#endif
#if !defined(ASN_DISABLE_JER_SUPPORT)
        0,
#endif
        SEQUENCE_constraint
    },
    asn_MBR_Submanoeuvre_1,
    5,	/* Elements count: now 5 */
    &asn_SPC_Submanoeuvre_specs_1
};