/** @file
 *
 * @brief Utensil functions for Core to Core
 *
 * @details This module contains the Core to Core utensil functions.
 *
 * @copyright
 * Copyright 2010-2017.
 * This software is protected Intellectual Property and may only be used
 * according to the license agreement.
 */

#ifndef GOAL_CTC_UTENSIL_H
#define GOAL_CTC_UTENSIL_H

#include "goal_target_ctc.h"
#include "goal_types.h"


/****************************************************************************/
/* Defines */
/****************************************************************************/
#define GOAL_CTC_SEM_READ 0                     /**< semaphore for reading access */
#define GOAL_CTC_SEM_WRITE 1                    /**< semaphore for writing access */

#define GOAL_CTC_VERSION_SIZE 2                 /**< version size */
#define GOAL_CTC_IDENTIFIER "CC"                /**< the identifier to verify initialization */
#define GOAL_CTC_IDENTIFIER_SIZE (sizeof(GOAL_CTC_IDENTIFIER) - 1) /**< size of the identifier */
#define CTC_FIFO_COUNT (1 << GOAL_CONFIG_CTC_FIFO_CNT_EXPONENT) /**< number of fifo elemets */


/****************************************************************************/
/* Typedefs */
/****************************************************************************/
typedef enum {
    GOAL_CTC_V_MAJOR_IDX = 0,                   /**< majors version index */
    GOAL_CTC_V_MINOR_IDX,                       /**< minors version index */
} GOAL_CTC_V_IDX_T;

typedef enum {
    GOAL_CTC_CREATE_PRECIS_INV = 0,             /**< invalid */
    GOAL_CTC_CREATE_PRECIS_NONE,                /**< create no precis */
    GOAL_CTC_CREATE_PRECIS_WRITE,               /**< create write precis */
    GOAL_CTC_CREATE_PRECIS_ALL,                 /**< create both precis */
} GOAL_CTC_CREATE_PRECIS_T;

/**< initializing states */
typedef enum {
    GOAL_CTC_STATUS_RESET = 0,                  /**< reset state */
    GOAL_CTC_STATUS_WRITE,                      /**< write version */
    GOAL_CTC_STATUS_READ,                       /**< read the foreign version */
    GOAL_CTC_STATUS_VALID,                      /**< validate the own and partner state */
    GOAL_CTC_STATUS_OK,                         /**< ctc is ready */
} GOAL_CTC_STATUS_STATES_T;

/**< message flag states */
typedef enum {
    GOAL_CTC_PURE_FLG_REQ = 0,                  /**< request message */
    GOAL_CTC_PURE_FLG_RES,                      /**< response message */
    GOAL_CTC_PURE_FLG_DATA,                     /**< only data buffer */
    GOAL_CTC_PURE_FLG_INV,                      /**< invalid */
} GOAL_CTC_PURE_FLG_T;

/**< cyclic message status */
typedef enum {
    GOAL_CTC_CHN_PURE_STATUS_UNASSIGNED = 0,    /**< pure channels has not be signed */
    GOAL_CTC_CHN_PURE_STATUS_CYCLIC,            /**< pure channels is cyclic */
    GOAL_CTC_CHN_PURE_STATUS_NOT_CYCLIC,        /**< pure channels is not cyclic */
} GOAL_CTC_CHN_PURE_STATUS_T;

/**< single entry of precis fifo */
typedef struct GOAL_CTC_DATA_ENTRY_T {
    uint32_t page;                              /**< message page (this might become an offset value) */
    uint16_t len;                               /**< data length */
} GOAL_CTC_DATA_ENTRY_T;

/**< precis message */
typedef GOAL_TARGET_PACKED_PRE struct {
    uint32_t channelId_be32;                    /**< pure channel ID */
    uint32_t flgMsg_be32;                       /**< message flag */
    uint32_t offset_be32;                       /**< offset to data */
    uint16_t len_be16;                          /**< data length */
    GOAL_CTC_DATA_ENTRY_T pEntry;               /**< precis entry */
} GOAL_TARGET_PACKED GOAL_CTC_PRECIS_MSG_T;

/**< precis of memory data */
typedef GOAL_TARGET_PACKED_PRE struct GOAL_CTC_PRECIS_T {
    uint32_t offsetNext_be32;                   /**< offset to next entry */
    uint32_t channelId_be32;                    /**< pure channel ID */
    uint32_t idxWr_be32;                        /**< write index */
    uint32_t idxRd_be32;                        /**< read index */
    uint32_t idxSw_be32;                        /**< swap index */
    GOAL_CTC_DATA_ENTRY_T ctcFifo[CTC_FIFO_COUNT]; /**< ctc fifo */
    uint32_t data;                              /**< first pure channel data - note the alignment! */
} GOAL_TARGET_PACKED GOAL_CTC_PRECIS_T;

/**< management part of the memory map */
typedef GOAL_TARGET_PACKED_PRE struct {
    char identifier[GOAL_CTC_IDENTIFIER_SIZE];  /**< notify initialized Buffer */
    uint8_t ctcVersion[GOAL_CTC_VERSION_SIZE];  /**< CTC version */
    uint32_t offsetPrecis_be32;                 /**< offset to first precis entry */
}  GOAL_TARGET_PACKED GOAL_CTC_MEMORY_MAP;

/**< ctc configuration
 * The address of the memory segment is NULL per default. The board has to
 * edit this value, if there is SHM available.
 * On e.g. SPI this value has to be keeped as NULL. The local RAM segment will
 * be allocated with the config len */
typedef struct GOAL_CTC_CONFIG_T {
    uint8_t *pMem;                              /**< memory segment */
    unsigned int len;                           /**< length of shared memory */
    GOAL_BOOL_T flgMemInit;                     /**< flag for memory initialization */
    uint16_t cntAck;                            /**< number of acknowledgements */
    uint32_t pureChnCnt;                        /**< number of pure channels */
    uint32_t pageCnt;                           /**< number of pages */
    uint32_t pageSize;                          /**< size of pages */
    uint8_t prioCnt;                            /**< number of priorities */
    unsigned int fragmSize;                     /**< number of acknowledges */
} GOAL_CTC_CONFIG_T;

typedef GOAL_STATUS_T (* GOAL_CTC_MEM_CREATE)(
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< configuration */
    unsigned int lenMin,                        /**< minimum length of memory */
    GOAL_CTC_CREATE_PRECIS_T *pCreatePrecis   /**< create precis information */
);

typedef GOAL_STATUS_T (* GOAL_CTC_PURE_CHN_INIT)(
    void *pArg                                  /**< function argument */
);

typedef GOAL_STATUS_T (* GOAL_CTC_DATA_WRITE)(
    unsigned int usageId,                       /**< usage ID */
    GOAL_CTC_PURE_FLG_T flgMsg,                 /**< message flag */
    uint32_t offset,                            /**< message offset */
    uint8_t *pData,                             /**< message data */
    uint16_t len                                /**< message length */
);

typedef struct {
    struct GOAL_CTC_CONFIG_T *pConfig;          /**< configuration */
    GOAL_CTC_MEMORY_MAP *pWrite;                /**< write section of memory */
    GOAL_CTC_MEMORY_MAP *pRead;                 /**< read section of memory */
    GOAL_CTC_STATUS_STATES_T state;             /**< ctc init state */
    GOAL_CTC_DATA_WRITE fctDataWrite;           /**< write function */
} GOAL_CTC_UTIL_T;


/****************************************************************************/
/* Makros */
/****************************************************************************/
#define CTC_PRECIS_ADDR(offsetPrecis) ((0 == offsetPrecis) ? (NULL) : ((GOAL_CTC_PRECIS_T *) (((uint8_t *) &offsetPrecis) + GOAL_be32toh(offsetPrecis))))


/****************************************************************************/
/* Prototypes */
/****************************************************************************/
void goal_ctcUtilPrecisMsgSet(
    GOAL_CTC_PRECIS_MSG_T *pMsgPrecis,          /**< precis message buffer */
    uint32_t channelId,                         /**< pure channel ID */
    uint32_t offset,                            /**< offset to data */
    uint16_t len,                               /**< data length */
    GOAL_CTC_DATA_ENTRY_T *pEntry,              /**< precis entry */
    GOAL_CTC_PURE_FLG_T flgMsg                  /**< message flag */
);

void goal_ctcUtilPrecisMsgGet(
    GOAL_CTC_PRECIS_MSG_T *pMsgPrecis,          /**< precis message buffer */
    uint32_t *pChannelId,                       /**< pure channel ID */
    uint32_t *pOffset,                          /**< offset to data */
    uint16_t *pLen,                             /**< data length */
    GOAL_CTC_DATA_ENTRY_T *pEntry,              /**< precis entry */
    GOAL_CTC_PURE_FLG_T *pFlgMsg                /**< message flag */
);

GOAL_STATUS_T goal_ctcUtilMemInit(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    uint8_t *pCtcVersion,                       /**< ctc version */
    GOAL_CTC_CREATE_PRECIS_T *pCreatePrecis,    /**< create precis information */
    GOAL_CTC_MEM_CREATE fctMemCreate            /**< function for creating memory */
);

GOAL_STATUS_T goal_ctcUtilPureChnInit(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    GOAL_CTC_CREATE_PRECIS_T createPrecis,      /**< create precis information */
    GOAL_CTC_PURE_CHN_INIT fctPureInit,         /**< function for pure channel init */
    void *pFctPureInitArg                       /**< argument for pure channel init */
);

void goal_ctcUtilPrecisInit(
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< configuration */
    GOAL_CTC_MEMORY_MAP *pSection               /**< section of memory */
);

GOAL_STATUS_T goal_ctcUtilPrecisGet(
    GOAL_CTC_PRECIS_T **ppPrecis,               /**< precis reference */
    GOAL_CTC_MEMORY_MAP *pSectionMap,           /**< read or write section of memory */
    uint32_t channelId                          /**< pure channel ID */
);

void goal_ctcUtilFifoWriteAdd(
    struct GOAL_CTC_PRECIS_T *pPrecis,          /**< channel precis */
    GOAL_CTC_DATA_ENTRY_T *pFifoEntry,          /**< fifo entry */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
);

void goal_ctcUtilFifoReadInc(
    struct GOAL_CTC_PRECIS_T *pPrecis,          /**< channel precis */
    uint32_t idxRdFifoNew,                      /**< new read index */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
);

GOAL_STATUS_T goal_ctcUtilAlloc(
    GOAL_CTC_DATA_ENTRY_T *pDataAlloc,          /**< alloc data for write */
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< configuration */
    GOAL_CTC_PRECIS_T *pPrecis,                 /**< pure channel precis */
    uint16_t len,                               /**< allocation length */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
);

GOAL_STATUS_T goal_ctcUtilPureStatus(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    unsigned int usageId                        /**< usage ID */
);

GOAL_STATUS_T goal_ctcUtilVersionValid(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    uint8_t *pCtcVersion,                       /**< own ctc version */
    GOAL_CTC_V_IDX_T versionIdx                 /**< version length */
);

GOAL_STATUS_T goal_ctcUtilSetup(
    uint32_t channelId,                         /**< pure channel ID */
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
);
#endif /* GOAL_CTC_UTENSIL_H */
