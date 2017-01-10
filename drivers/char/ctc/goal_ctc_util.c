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

#include "goal_ctc_util.h"


/****************************************************************************/
/* Defines */
/****************************************************************************/
/**< construction of precis data */
#define GOAL_CTC_PRECIS_ENTRY_PAGE_INV ((uint32_t) -1) /**< define for invalid pages */
#define GOAL_CTC_PRECIS_ENTRY_LEN_INV ((uint16_t) -1) /**< define for invalid length pages */

#define GOAL_CTC_CYCLIC_BUFFER 4                /**< number of segments for cyclic data */


/****************************************************************************/
/* Prototypes */
/****************************************************************************/
static void goal_ctcUtilCalcPages(
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< MI configuration */
    uint16_t len,                               /**< data length */
    unsigned int ctcIdx,                        /**< page index */
    unsigned int *pReqPages,                    /**< required pages */
    unsigned int *pOffsetIdx                    /**< offset to write index */
);

static GOAL_STATUS_T goal_ctcUtilMemAssign(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    GOAL_CTC_CREATE_PRECIS_T createPrecis       /**< create precis information */
);

static void goal_ctcUtilMemSizeGet(
    unsigned int *pLenMin,                      /**< reference for minimum length of memory */
    struct GOAL_CTC_CONFIG_T *pConfig           /**< configuration */
);


/****************************************************************************/
/** Set Precis message
 *
 * This function prepares a message for precis usage by packing the data into
 * the precis structure. The conversation of the endianness is done if
 * necessary.
 */
void goal_ctcUtilPrecisMsgSet(
    GOAL_CTC_PRECIS_MSG_T *pMsgPrecis,          /**< precis message buffer */
    uint32_t channelId,                         /**< pure channel ID */
    uint32_t offset,                            /**< offset to data */
    uint16_t len,                               /**< data length */
    GOAL_CTC_DATA_ENTRY_T *pFifoEntry,          /**< fifo entry */
    GOAL_CTC_PURE_FLG_T flgMsg                  /**< message flag */
)
{
    pMsgPrecis->channelId_be32 = GOAL_htobe32(channelId);
    pMsgPrecis->offset_be32 = GOAL_htobe32(offset);
    pMsgPrecis->len_be16 = GOAL_htobe16(len);
    pMsgPrecis->flgMsg_be32 = GOAL_htobe32((uint32_t) flgMsg);
    if (NULL != pFifoEntry) {
        pMsgPrecis->pEntry.page = pFifoEntry->page;
        pMsgPrecis->pEntry.len = pFifoEntry->len;
    } else {
        pMsgPrecis->pEntry.page = GOAL_CTC_PRECIS_ENTRY_PAGE_INV;
        pMsgPrecis->pEntry.len = GOAL_CTC_PRECIS_ENTRY_LEN_INV;
    }
}


/****************************************************************************/
/** Get precis message
 *
 * This function reads informations from the precis buffer.
 */
void goal_ctcUtilPrecisMsgGet(
    GOAL_CTC_PRECIS_MSG_T *pMsgPrecis,          /**< precis message buffer */
    uint32_t *pChannelId,                       /**< pure channel ID */
    uint32_t *pOffset,                          /**< offset to data */
    uint16_t *pLen,                             /**< data length */
    GOAL_CTC_DATA_ENTRY_T *pFifoEntry,          /**< fifo entry */
    GOAL_CTC_PURE_FLG_T *pFlgMsg                /**< message flag */
)
{
    if (NULL != pChannelId) {
        *pChannelId = GOAL_be32toh(pMsgPrecis->channelId_be32);
    }
    if (NULL != pOffset) {
        *pOffset = GOAL_be32toh(pMsgPrecis->offset_be32);
    }
    if (NULL != pLen) {
        *pLen = GOAL_be16toh(pMsgPrecis->len_be16);
    }
    if (NULL != pFifoEntry) {
        pFifoEntry->page = pMsgPrecis->pEntry.page;
        pFifoEntry->len = pMsgPrecis->pEntry.len;
    }
    if (NULL != pFlgMsg) {
        *pFlgMsg = (GOAL_CTC_PURE_FLG_T) GOAL_be32toh(pMsgPrecis->flgMsg_be32);
    }
}


/****************************************************************************/
/** Calculate Pages
 *
 * This function calculates the required pages and the index offset based on
 * the data length and the index.
 * The required pages doesn't include the offset!
 *
 */
static void goal_ctcUtilCalcPages(
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< MI configuration */
    uint16_t len,                               /**< data length */
    unsigned int ctcIdx,                        /**< page index */
    unsigned int *pReqPages,                    /**< required pages */
    unsigned int *pOffsetIdx                    /**< offset to write index */
)
{
    unsigned int reqPages;                      /* required pages */

    /* calculate the necessary pages */
    reqPages = (unsigned int) (len / pConfig->pageSize);
    if (len % pConfig->pageSize) {
        reqPages++;
    }

    /* The data will not be splitted at the end of the cyclic buffer. Instead
     * they'll be written to the first page */
    /* Check if the new data will fit onto the buffer */
    if (pConfig->pageCnt < (ctcIdx + reqPages)) {
        /* calculate offset */
        *pOffsetIdx = (unsigned int) (pConfig->pageCnt - ctcIdx);
    } else {
        *pOffsetIdx = 0;
    }

    /* hand back the calculated required pages */
    *pReqPages = reqPages;
}


/****************************************************************************/
/** Initialize the memory for ctc usage
 *
 * This function will initialize the memory for core to core usage. This
 * includes the eventual creation of a memory segment, validation and
 * assignment of pure channels.
 *
 * @retval GOAL_OK successful
 * @retval other failed
 */
GOAL_STATUS_T goal_ctcUtilMemInit(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    uint8_t *pCtcVersion,                       /**< ctc version */
    GOAL_CTC_CREATE_PRECIS_T *pCreatePrecis,    /**< create precis information */
    GOAL_CTC_MEM_CREATE fctMemCreate            /**< function for creating memory */
)
{
    GOAL_STATUS_T res = GOAL_OK;                /* result */
    unsigned int lenMin;                        /* minimum length of memory for one direction */

    /* create the memory section if it hasn't been linked before to the ctc configuration */
    if (NULL != fctMemCreate) {
        /* get the minimum memory size */
        goal_ctcUtilMemSizeGet(&lenMin, pUtil->pConfig);

        /* call the memory create function */
        res = fctMemCreate(pUtil->pConfig, lenMin, pCreatePrecis);
    }

    /* assign the read and write section */
    if (GOAL_RES_OK(res)) {
        res = goal_ctcUtilMemAssign(pUtil, *pCreatePrecis);
    }

    if (GOAL_RES_OK(res)) {
        /* init the write section */
        /* Copy the version number of the core to core system on the buffer, so that a possible */
        /* incompatible version trying to communicate may be identified */
        GOAL_MEMCPY((void *) pUtil->pWrite->ctcVersion, pCtcVersion, GOAL_CTC_VERSION_SIZE);
    }

    return res;
}


/****************************************************************************/
/** Assign the memory regions
 *
 * This function will split the memory and assign the parts into read and
 * write sections next to the precis of the pure channels.
 * The design of the memory map is:
 * | AC write memory map |
 * | CC write memory map |
 * | write precis AC / read precis CC |
 * | write precis CC / read precis AC |
 *
 * The way of of initializing the link of memory map to first precis can be:
 * GOAL_CTC_CREATE_PRECIS_NONE - it has been initialized before, e.g. by
 * kernel
 * GOAL_CTC_CREATE_PRECIS_WRITE - shared memory, initialize only the write
 * section, e.g. on DPRAM
 * GOAL_CTC_CREATE_PRECIS_ALL - no shared memory, e.g. on SPI
 */
static GOAL_STATUS_T goal_ctcUtilMemAssign(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    GOAL_CTC_CREATE_PRECIS_T createPrecis       /**< create precis information */
)
{
    uint32_t offsetPrecis;                      /* precis offset */
    GOAL_CTC_PRECIS_T *pPrecis = NULL;          /* conted pointer */

    if (GOAL_CTC_CREATE_PRECIS_INV == createPrecis) {
        goal_logErr("Invalid create precis information.");
        return GOAL_ERR_PARAM;
    }

#if GOAL_CTC_CC == 1
        pUtil->pRead = (GOAL_CTC_MEMORY_MAP *) pUtil->pConfig->pMem;
        pUtil->pWrite = (GOAL_CTC_MEMORY_MAP *) (pUtil->pConfig->pMem + sizeof(GOAL_CTC_MEMORY_MAP));

        if (GOAL_CTC_CREATE_PRECIS_NONE != createPrecis) {
            if (GOAL_CTC_CREATE_PRECIS_ALL == createPrecis) {
                /* create the first offset precis of the read section */
                offsetPrecis = (uint32_t) ((uint8_t *) &(pUtil->pRead->offsetPrecis_be32) - pUtil->pConfig->pMem);
                offsetPrecis = (2 * sizeof(GOAL_CTC_MEMORY_MAP)) - offsetPrecis;

                /* assign the first channel precis */
                pUtil->pRead->offsetPrecis_be32 = GOAL_htobe32(offsetPrecis);
            }

            /* clear the management section in case of a configured memory section */
            GOAL_MEMSET(pUtil->pWrite, 0, sizeof(GOAL_CTC_MEMORY_MAP));

            /* create the first offset precis of the write section */
            offsetPrecis = (uint32_t) ((uint8_t *) &(pUtil->pWrite->offsetPrecis_be32) - pUtil->pConfig->pMem);
            offsetPrecis = ((2 * sizeof(GOAL_CTC_MEMORY_MAP)) + (pUtil->pConfig->len >> 1)) - offsetPrecis;

            /* assign the first channel precis */
            pUtil->pWrite->offsetPrecis_be32 = GOAL_htobe32(offsetPrecis);

            pPrecis = CTC_PRECIS_ADDR(pUtil->pWrite->offsetPrecis_be32);
        } else {
            pUtil->state = GOAL_CTC_STATUS_OK;
        }
#elif GOAL_CTC_AC == 1
        pUtil->pRead = (GOAL_CTC_MEMORY_MAP *) (pUtil->pConfig->pMem + sizeof(GOAL_CTC_MEMORY_MAP));
        pUtil->pWrite = (GOAL_CTC_MEMORY_MAP *) pUtil->pConfig->pMem;

        /* create the first offset precis of the read section if required */
        if (GOAL_CTC_CREATE_PRECIS_NONE != createPrecis) {
            if (GOAL_CTC_CREATE_PRECIS_ALL == createPrecis) {
                offsetPrecis = (uint32_t) ((uint8_t *) &(pUtil->pRead->offsetPrecis_be32) - pUtil->pConfig->pMem);
                offsetPrecis = ((2 * sizeof(GOAL_CTC_MEMORY_MAP)) + (pUtil->pConfig->len >> 1)) - offsetPrecis;
                /* assign the first channel precis */
                pUtil->pRead->offsetPrecis_be32 = GOAL_htobe32(offsetPrecis);
            }

            /* clear the section in case of a configured memory section */
            GOAL_MEMSET(pUtil->pWrite, 0, sizeof(GOAL_CTC_MEMORY_MAP));

            /* create the first offset precis of the write section */
            offsetPrecis = (uint32_t) ((uint8_t *) &(pUtil->pWrite->offsetPrecis_be32) - pUtil->pConfig->pMem);
            offsetPrecis = (2 * sizeof(GOAL_CTC_MEMORY_MAP)) - offsetPrecis;
            /* assign the first channel precis */
            pUtil->pWrite->offsetPrecis_be32 = GOAL_htobe32(offsetPrecis);

            pPrecis = CTC_PRECIS_ADDR(pUtil->pWrite->offsetPrecis_be32);
        } else {
            pUtil->state = GOAL_CTC_STATUS_OK;
        }
#else
#error either GOAL_CTC_CC or GOAL_CTC_AC must be defined
#endif

    if (pPrecis) {
        /* clear the precis section in case of a configured memory section */
        GOAL_MEMSET(pPrecis, 0, (pUtil->pConfig->len >> 1) - sizeof(GOAL_CTC_MEMORY_MAP));
    }

    return GOAL_OK;
}


/****************************************************************************/
/** Get the minimum memory size
 *
 * This function calculates the required minimum memory size. This includes
 * the map, the preciss and the cylic buffers for read and write section.
 *
 * @retval GOAL_OK successful
 * @retval other failed
 */
static void goal_ctcUtilMemSizeGet(
    unsigned int *pLenMin,                      /**< reference for minimum length of memory */
    struct GOAL_CTC_CONFIG_T *pConfig           /**< configuration */
)
{
    unsigned int lenMin;                        /* minimum length of memory */

    /* calculate the min length of the memory by basic map.. */
    lenMin = sizeof(GOAL_CTC_MEMORY_MAP);
    /* ..the precis for every channel.. */
    lenMin += (pConfig->pureChnCnt * sizeof(GOAL_CTC_PRECIS_T));
    /* ..the cyclic buffer for every channel.. */
    lenMin += (pConfig->pureChnCnt * pConfig->pageCnt * pConfig->pageSize);
    /* note the read section of the memory */
    lenMin *= 2;

    *pLenMin = lenMin;
}


/****************************************************************************/
/** Initialize the pure channels
 *
 * This function creates the precis of the pure channels and initialize it
 * afterwards. The memory region has to be initialized before.
 *
 * @retval GOAL_OK successful
 * @retval other failed
 */
GOAL_STATUS_T goal_ctcUtilPureChnInit(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    GOAL_CTC_CREATE_PRECIS_T createPrecis,      /**< create precis information */
    GOAL_CTC_PURE_CHN_INIT fctPureInit,         /**< function for pure channel init */
    void *pFctPureInitArg                       /**< argument for pure channel init */
)
{
    GOAL_STATUS_T res = GOAL_OK;                /* result */

    if (GOAL_CTC_CREATE_PRECIS_INV == createPrecis) {
        goal_logErr("Invalid create precis information.");
        return GOAL_ERR_PARAM;
    }

    /* create the precis of the read section if required */
    if (GOAL_CTC_CREATE_PRECIS_ALL == createPrecis) {
        goal_ctcUtilPrecisInit(pUtil->pConfig, pUtil->pRead);
    }

    /* create the precis of the write section if required */
    if ((GOAL_CTC_CREATE_PRECIS_ALL == createPrecis) || (GOAL_CTC_CREATE_PRECIS_WRITE == createPrecis)) {
        goal_ctcUtilPrecisInit(pUtil->pConfig, pUtil->pWrite);
    }
    if (NULL != fctPureInit) {
        res = fctPureInit(pFctPureInitArg);
    }
    return res;
}


/****************************************************************************/
/** Create the precis of a memory section
 *
 * This function creates the precis of a memory section.
 *
 * @retval GOAL_OK successful
 * @retval other failed
 */
void goal_ctcUtilPrecisInit(
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< configuration */
    GOAL_CTC_MEMORY_MAP *pSection               /**< section of memory */
)
{
    uint32_t offsetNextPrecis;                  /* offset to next precis */
    struct GOAL_CTC_PRECIS_T *pChnPrecis;       /* channel precis */
    unsigned cntChn;                            /* counter for channels */

    /* calculate the offset of one precis */
    offsetNextPrecis = sizeof(GOAL_CTC_PRECIS_T) + (pConfig->pageCnt * pConfig->pageSize);

    /* get the first precis of the write section */
    pChnPrecis = CTC_PRECIS_ADDR(pSection->offsetPrecis_be32);

    /* create the channel precis on the memory */
    for (cntChn = 0; cntChn < pConfig->pureChnCnt; cntChn++) {
        /* write the offset to the precis structure on memory */
        pChnPrecis->offsetNext_be32 = GOAL_htobe32(offsetNextPrecis);
        pChnPrecis->channelId_be32 = GOAL_htobe32(cntChn);
        pChnPrecis = CTC_PRECIS_ADDR(pChnPrecis->offsetNext_be32);
    }
}


/****************************************************************************/
/** Get Precis of a pure channel
 *
 * This function iterate to the precis of the pure channel.
 *
 * @retval GOAL_OK success
 * @retval other failure
 */
GOAL_STATUS_T goal_ctcUtilPrecisGet(
    GOAL_CTC_PRECIS_T **ppPrecis,               /**< precis reference */
    GOAL_CTC_MEMORY_MAP *pSectionMap,           /**< read or write section of memory */
    uint32_t channelId                          /**< pure channel ID */
)
{
    struct GOAL_CTC_PRECIS_T *pPrecisIdx;       /* precis index */

    /* select the first precis entry */
    pPrecisIdx = CTC_PRECIS_ADDR(pSectionMap->offsetPrecis_be32);

    for (; pPrecisIdx; pPrecisIdx = CTC_PRECIS_ADDR(pPrecisIdx->offsetNext_be32)) {
        if (channelId == GOAL_be32toh(pPrecisIdx->channelId_be32)) {
            *ppPrecis = pPrecisIdx;
            return GOAL_OK;
        }
    }

    return GOAL_ERROR;
}


/****************************************************************************/
/** Add fifo entry
 *
 * This function adds a new fifo entry to the precis.
 */
void goal_ctcUtilFifoWriteAdd(
    struct GOAL_CTC_PRECIS_T *pPrecis,          /**< channel precis */
    GOAL_CTC_DATA_ENTRY_T *pFifoEntry,          /**< fifo entry */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
)
{
    uint32_t idxFifoWr;                         /* fifo write index */
    int32_t cnt = 1;                            /* counter */
    uint32_t cntFifo = CTC_FIFO_COUNT;          /* number of used fifo elements */
    GOAL_BOOL_T flgDone = GOAL_FALSE;           /* adding write entry is done flag */

    /* add fifo entry to fifo list */
    idxFifoWr = GOAL_be32toh(pPrecis->idxWr_be32);
    GOAL_MEMCPY(&(pPrecis->ctcFifo[idxFifoWr]), pFifoEntry, sizeof(GOAL_CTC_DATA_ENTRY_T));

    /* udpate the swap index */
    pPrecis->idxSw_be32 = pPrecis->idxWr_be32;

    if (GOAL_CTC_CHN_PURE_STATUS_CYCLIC == cyclicState) {
        /* on cyclic channel, the counter can be increased up to three times,
         * because it may be possible to skip read and swap */
        cnt = GOAL_CTC_CYCLIC_BUFFER - 1;
        cntFifo = GOAL_CTC_CYCLIC_BUFFER;
    }

    /* increase the fifo write index, considering the overflow */
    for (; (!flgDone) && (0 < cnt); cnt--) {
        idxFifoWr = (idxFifoWr + 1) & (cntFifo - 1);
        if ((GOAL_be32toh(pPrecis->idxRd_be32) != idxFifoWr) && (GOAL_be32toh(pPrecis->idxSw_be32) != idxFifoWr)) {
            flgDone = GOAL_TRUE;
        }
    }

    if (!flgDone) {
        goal_logErr("Adding fifo entry failed!");
        return;
    }
    pPrecis->idxWr_be32 = GOAL_htobe32(idxFifoWr);
}


/****************************************************************************/
/** Increase read index
 *
 * This function increases the read index of the fifo. It is called by the MI.
 */
void goal_ctcUtilFifoReadInc(
    struct GOAL_CTC_PRECIS_T *pPrecis,          /**< channel precis */
    uint32_t idxRdFifoNew,                      /**< new read index */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
)
{
    uint32_t idxRdFifo;                         /* fifo read index */
    uint32_t idxWrFifo;                         /* fifo write index */
    uint32_t numUsed;                           /* used indexes */

    if (GOAL_CTC_CHN_PURE_STATUS_CYCLIC != cyclicState) {
        /* get the fifo read index */
        idxRdFifo = GOAL_be32toh(pPrecis->idxRd_be32);
        idxWrFifo = GOAL_be32toh(pPrecis->idxWr_be32);

        if (CTC_FIFO_COUNT <= idxRdFifoNew) {
            goal_logErr("New read index is out of range.");
            return;
        }

        /* validate the space on cyclic buffer */
        numUsed = CTC_FIFO_COUNT - (((CTC_FIFO_COUNT + idxRdFifo - idxWrFifo - 1)) % CTC_FIFO_COUNT);
        if (numUsed < ((CTC_FIFO_COUNT + idxRdFifoNew - idxRdFifo) % CTC_FIFO_COUNT)) {
            goal_logErr("Invalid read index received");
            return;
        }
        pPrecis->idxRd_be32 = GOAL_htobe32(idxRdFifoNew);
    }
}


/****************************************************************************/
/** Allocate a pure channel buffer
 *
 * This function allocates a pure channel buffer. Locking the precis has to
 * be done outside of this function.
 *
 * @retval GOAL_OK success
 * @retval other failure
 */
GOAL_STATUS_T goal_ctcUtilAlloc(
    GOAL_CTC_DATA_ENTRY_T *pDataAlloc,          /**< alloc data for write */
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< configuration */
    GOAL_CTC_PRECIS_T *pPrecis,                 /**< pure channel precis */
    uint16_t len,                               /**< allocation length */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
)
{
    uint32_t idxFifo;                           /* fifo index */
    uint32_t pageWr;                            /* write page */
    uint32_t pageRd;                            /* read page */
    uint32_t remPages;                          /* remaining pages */
    unsigned int idxOffset;                     /* offset to write index */
    unsigned int reqPages;                      /* required pages */

    /* null pointer validation */
    if ((NULL == pDataAlloc) || (NULL == pConfig) || (NULL == pPrecis)) {
        return GOAL_ERR_NULL_POINTER;
    }

    if (GOAL_CTC_CHN_PURE_STATUS_CYCLIC == cyclicState) {
        /* cyclic data - allocate a quarter of available memory */
        if (((pConfig->pageCnt * pConfig->pageSize) / 4) < len) {
            goal_logErr("The cyclic buffer is to small");
            return GOAL_ERROR;
        }

        idxFifo = GOAL_be32toh(pPrecis->idxWr_be32);
        pageWr = GOAL_be32toh(pPrecis->ctcFifo[idxFifo].page);
    }
    else {
        idxFifo = GOAL_be32toh(pPrecis->idxRd_be32);
        pageRd = GOAL_be32toh(pPrecis->ctcFifo[idxFifo].page);
        idxFifo = GOAL_be32toh(pPrecis->idxWr_be32);
        pageWr = GOAL_be32toh(pPrecis->ctcFifo[idxFifo].page);

        goal_ctcUtilCalcPages(pConfig, len, pageWr, &reqPages, &idxOffset);

        /* calculate the remaining pages on the buffer */
        remPages = (pConfig->pageCnt - 1) - pageWr + pageRd;
        remPages &= (pConfig->pageCnt - 1);

        /* check if there are enough free entries on the buffer */
        if (remPages < (reqPages + idxOffset)) {
            goal_logDbg("No free pages for ctc left.");
            return GOAL_ERR_FULL;
        }

        /* add the offset and consider the overflow */
        pageWr += idxOffset;
        pageWr &= (pConfig->pageCnt - 1);

        /* Save the next free page to the next fifo element. This is necessary to get the
         * pageWr on the next goal_ctcPureAlloc call. Note: there is always a free fifo entry
         * between read and write index. The ctc fifo write index will be increased, when announcing the buffer */
        idxFifo = (idxFifo + 1) & (CTC_FIFO_COUNT - 1);
        pPrecis->ctcFifo[idxFifo].page = GOAL_htobe32((pageWr + reqPages) & (pConfig->pageCnt - 1));
    }
    /* save the alloc information */
    pDataAlloc->page = GOAL_htobe32(pageWr);
    pDataAlloc->len = GOAL_htobe16(len);

    return GOAL_OK;
}


/****************************************************************************/
/** Pure channel status
 *
 * This function validates the states of the memory structures for pure
 * channel communication. The following states are used:
 *
 * GOAL_CTC_STATUS_RESET:
 * The identifier on the memory read section is used to request the foreign
 * version. It has to be set to GOAL_CTC_IDENTIFIER if the version check was
 * successful.
 *
 * GOAL_CTC_STATUS_WRITE:
 * While the identifier on the memory write section is not GOAL_CTC_IDENTIFIER
 * the version has to send to the destination.
 *
 * GOAL_CTC_STATUS_READ:
 * Validate the foreign version. If the version is okay, set the identifier
 * on the memory read section to GOAL_CTC_IDENTIFIER and send it to the other
 * core.
 *
 * GOAL_CTC_STATUS_VALID:
 * Check the identifier on read and write section. They have to be
 * GOAL_CTC_IDENTIFIER.
 *
 * GOAL_CTC_STATUS_OK:
 * Version check is done with success. Cyclic validation of the identifier on
 * the write section is necessary for resetting the ctc status.
 *
 * @retval GOAL_OK successful
 * @retval other failed
 */
GOAL_STATUS_T goal_ctcUtilPureStatus(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    unsigned int usageId                        /**< usage ID */
)
{
    GOAL_STATUS_T res = GOAL_ERROR;             /* result */

    switch (pUtil->state) {
        /* set and write the read ident to 0 to sign the start */
        case GOAL_CTC_STATUS_RESET:
            /* reset the read identifier */
            GOAL_MEMSET(pUtil->pRead->identifier, 0, GOAL_CTC_IDENTIFIER_SIZE);
            if (NULL != pUtil->fctDataWrite) {
                res = pUtil->fctDataWrite(usageId,
                        GOAL_CTC_PURE_FLG_DATA,
                        (uint32_t) ((uint8_t *) pUtil->pRead->identifier - (uint8_t *) pUtil->pConfig->pMem),
                        (uint8_t *) pUtil->pRead->identifier,
                        GOAL_CTC_IDENTIFIER_SIZE);
                if (GOAL_RES_ERR(res)) {
                    goal_logErr("Sending read identifier FAILED");
                    return GOAL_ERR_NOT_INITIALIZED;
                }
            }
            pUtil->state = GOAL_CTC_STATUS_WRITE;
            goal_logDbg("new ctc init state: #%u on ctc MI ID %u", pUtil->state, usageId);
        break;

        /* check the write identifier (set by other core during GOAL_CTC_STATUS_RESET) */
        case GOAL_CTC_STATUS_WRITE:
            if (0 != GOAL_MEMCMP(pUtil->pWrite->identifier, GOAL_CTC_IDENTIFIER, GOAL_CTC_IDENTIFIER_SIZE)) {
                /* send write map */
                if (NULL != pUtil->fctDataWrite) {
                    res = pUtil->fctDataWrite(usageId,
                            GOAL_CTC_PURE_FLG_DATA,
                            (uint32_t) ((uint8_t *) pUtil->pWrite->ctcVersion - (uint8_t *) pUtil->pConfig->pMem),
                            (uint8_t *) pUtil->pWrite->ctcVersion,
                            GOAL_CTC_VERSION_SIZE + sizeof(pUtil->pWrite->offsetPrecis_be32));
                    if (GOAL_RES_ERR(res)) {
                        goal_logErr("Sending ctc version FAILED on ctc MI ID %u", usageId);
                        return GOAL_ERR_NOT_INITIALIZED;
                    }
                }
            }
            pUtil->state = GOAL_CTC_STATUS_READ;
            goal_logDbg("new ctc init state: #%u on ctc MI ID %u", pUtil->state, usageId);
        break;

        case GOAL_CTC_STATUS_READ:
            if (0 != GOAL_MEMCMP(pUtil->pRead->identifier, GOAL_CTC_IDENTIFIER, GOAL_CTC_IDENTIFIER_SIZE)) {
                /* validate the version */
                res = goal_ctcUtilVersionValid(pUtil, pUtil->pWrite->ctcVersion, GOAL_CTC_V_MAJOR_IDX);
                if (GOAL_RES_OK(res)) {
                    /* acknowledge identifier to read section */
                    GOAL_MEMCPY(pUtil->pRead->identifier, GOAL_CTC_IDENTIFIER, GOAL_CTC_IDENTIFIER_SIZE);
                }

                if (NULL != pUtil->fctDataWrite) {
                    res = pUtil->fctDataWrite(usageId,
                                    GOAL_CTC_PURE_FLG_DATA,
                                    (uint32_t) ((uint8_t *) pUtil->pRead->identifier - (uint8_t *) pUtil->pConfig->pMem),
                                    (uint8_t *) pUtil->pRead->identifier,
                                    GOAL_CTC_IDENTIFIER_SIZE);
                    if (GOAL_RES_ERR(res)) {
                        goal_logErr("Sending read identifier FAILED on ctc MI ID %u", usageId);
                        /* clear the identifier for next cycle */
                        GOAL_MEMSET(pUtil->pRead->identifier, 0, GOAL_CTC_IDENTIFIER_SIZE);
                        return GOAL_ERR_NOT_INITIALIZED;
                    }
                }

            }
            pUtil->state = GOAL_CTC_STATUS_VALID;
            goal_logDbg("new ctc init state: #%u on ctc MI ID %u", pUtil->state, usageId);
        break;

        case GOAL_CTC_STATUS_VALID:
            if (0 == GOAL_MEMCMP(pUtil->pWrite->identifier, pUtil->pRead->identifier, GOAL_CTC_IDENTIFIER_SIZE)) {
                if (0 == GOAL_MEMCMP(pUtil->pWrite->identifier, GOAL_CTC_IDENTIFIER, GOAL_CTC_IDENTIFIER_SIZE)) {
                    /* set the ready flag to enable the communication on pure channels */
                    res = goal_ctcUtilVersionValid(pUtil, pUtil->pWrite->ctcVersion, GOAL_CTC_V_MAJOR_IDX);
                    if (GOAL_RES_ERR(res)) {
                        goal_logWarn("ctc version: %u", pUtil->pWrite->ctcVersion[GOAL_CTC_V_MAJOR_IDX]);
                        goal_logWarn("    : found invalid foreign version %u for ctc MI ID %u", pUtil->pRead->ctcVersion[GOAL_CTC_V_MAJOR_IDX], usageId);
                    }
                    else {
                        pUtil->state = GOAL_CTC_STATUS_OK;
                        goal_logDbg("new ctc init state: #%u on ctc MI ID %u", pUtil->state, usageId);
                        break;
                    }
                }
            }
            pUtil->state = GOAL_CTC_STATUS_WRITE;
            goal_logDbg("new ctc init state: #%u on ctc MI ID %u", pUtil->state, usageId);
        break;

        case GOAL_CTC_STATUS_OK:
            if (0 != GOAL_MEMCMP(pUtil->pWrite->identifier, pUtil->pRead->identifier, GOAL_CTC_IDENTIFIER_SIZE)) {
                pUtil->state = GOAL_CTC_STATUS_RESET;
                goal_logDbg("new ctc init state: #%u on ctc MI ID %u", pUtil->state, usageId);
            }
        break;

        default:
            goal_logErr("Error");
        break;
    }

    /* quit only with GOAL_OK, if the required state has been reached */
    if (pUtil->state == GOAL_CTC_STATUS_OK) {
        return GOAL_OK;
    }

    return GOAL_ERR_NOT_INITIALIZED;
}


/****************************************************************************/
/** Validate a part of CTC version
 *
 * This function validates the ctc version by a selectable length.
 * During pure channel boot-up, only the major version number is checked,
 * because the kernel module minor number may be different from the CTC minor
 * number.
 * When handshaking the ctc channels, the complete ctc version is checked.
 *
 * @retval GOAL_OK success
 * @retval other failure
 */
GOAL_STATUS_T goal_ctcUtilVersionValid(
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    uint8_t *pCtcVersion,                       /**< own ctc version */
    GOAL_CTC_V_IDX_T versionIdx                 /**< version length */
)
{
    unsigned int len = 0;                       /* version length */

    switch (versionIdx) {
        case GOAL_CTC_V_MAJOR_IDX:
            len = 1;
        break;

        case GOAL_CTC_V_MINOR_IDX:
            len = 2;
        break;

        default:
            return GOAL_ERROR;
    }

    /* validate the version */
    if (0 == GOAL_MEMCMP(pCtcVersion, pUtil->pRead->ctcVersion, len)) {
        return GOAL_OK;
    }

    return GOAL_ERROR;
}


/****************************************************************************/
/** Utensil setup
 *
 * This function setups the precis of the named pure channel for cyclic data
 * by splitting the memory into GOAL_CTC_CYCLIC_BUFFER segments.
 *
 *
 * @retval GOAL_OK success
 * @retval other failure
 */
GOAL_STATUS_T goal_ctcUtilSetup(
    uint32_t channelId,                         /**< pure channel ID */
    GOAL_CTC_UTIL_T *pUtil,                     /**< general utility */
    GOAL_CTC_CHN_PURE_STATUS_T cyclicState      /**< cyclic channel status */
)
{
    uint32_t cnt;                               /* counter */
    GOAL_STATUS_T res = GOAL_OK;                /* result */
    GOAL_CTC_PRECIS_T *pPrecis = NULL;          /* conted pointer */

    /* setup the precis for cyclic messages */
    if (GOAL_CTC_CHN_PURE_STATUS_CYCLIC == cyclicState) {
        res = goal_ctcUtilPrecisGet(&pPrecis, pUtil->pWrite, channelId);
        if (GOAL_RES_OK(res)) {
            for (cnt = 0; cnt < GOAL_CTC_CYCLIC_BUFFER; cnt++) {
                pPrecis->ctcFifo[cnt].page = GOAL_htobe32(cnt * ((pUtil->pConfig->pageCnt) / GOAL_CTC_CYCLIC_BUFFER));
            }
        }
    }

    return res;
}
