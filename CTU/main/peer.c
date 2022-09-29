#include <assert.h>
#include <string.h>

#include "include/peer.h"

#include "ble_central.h"



/* Memory pool data structure definitions */
static void *peer_svc_mem;
static struct os_mempool peer_svc_pool;

static void *peer_chr_mem;
static struct os_mempool peer_chr_pool;

static void *peer_dsc_mem;
static struct os_mempool peer_dsc_pool;

static void *peer_mem;
static struct os_mempool peer_pool;

/* Static function declarations */
static struct peer_svc *peer_svc_find_range(struct peer *peer, uint16_t attr_handle);
static struct peer_svc *peer_svc_find(struct peer *peer, uint16_t svc_start_handle,
              struct peer_svc **out_prev);
int peer_svc_is_empty(const struct peer_svc *svc);

uint16_t chr_end_handle(const struct peer_svc *svc, const struct peer_chr *chr);
int chr_is_empty(const struct peer_svc *svc, const struct peer_chr *chr);
static struct peer_chr *peer_chr_find(const struct peer_svc *svc, uint16_t chr_def_handle,
              struct peer_chr **out_prev);
static void peer_disc_chrs(struct peer *peer);

static int peer_dsc_disced(uint16_t conn_handle, const struct ble_gatt_error *error,
                uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                void *arg);

static const char* TAG = "PEER";

static uint8_t NUM_CRU = 0;
static uint8_t NUM_AUX_CTU = 0;

//defined in ble_central.c
extern const ble_uuid_t *wpt_svc_uuid;

/**
 * @brief Gives the total number of connected CRU
 *
*/
uint8_t peer_get_NUM_CRU(void) {
    return NUM_CRU;
}

/**
 * @brief Gives the total number of connected Auxiliary CTU
 *
*/
uint8_t peer_get_NUM_AUX_CTU(void) {
    return NUM_AUX_CTU;
}


/**
 * @brief Function that returns the position of the CRU
 * 
 * @param conn_handle The peer's connection handle
 * @return 0 if no localization process has been completed yet, position therwise.
*/
int get_peer_position(uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    return peer->position;
}

/**
 * @brief Function that returns if a localization process is being done by any peer
 * @details This function searches through a linked list of peers
 *          and tries to check if any peers is currently doing the localization process
 * 
 * @return 0 if no, 1 therwise.
*/
bool current_localization_process(void)
{
    struct peer *peer;
    bool loc = 0;

    SLIST_FOREACH(peer, &peers, next) {
        if (peer->localization_process == 1) {
            loc = 1;
        }
    }
    return loc;
}

/**
 * @brief Determines if there's only one peer connected
 * @details This function simply determines if the
 *          total number of peers is exactly 1.
 * 
 * @return true if the peer is alone, false otherwise.
*/
bool is_peer_alone(void)
{
    return NUM_CRU == 1;
}

uint8_t low_power_find(void)
{
    uint8_t n;
    for(n=0; n<4; n++)
    {
        if(low_power_pads[n])
        {
            break;
        }
    }
    return n;
}

/**
 * @brief Boolean function for charging state
 * @details This function asserts if a specified peer is charging or not.
 * 
 * @param peer The analyzed peer structure
 * @return True if peer is charging in full-power mode, false in any other case
*/
bool CTU_is_peer_charging(struct peer *peer)
{
    return full_power_pads[peer->position-1];
}

/**
 * @brief Boolean function for full-power charge state
 * @details This function asserts if at least one charging pad is in full-power mode.
 * 
 * @return True if peer is yes, false in any other case
*/
bool CTU_is_charging(void)
{
    if(full_power_pads[0] || full_power_pads[1] || full_power_pads[2] || full_power_pads[3])
    {
        return 1;
    } else 
    {
        return 0;
    }
}

/**
 * @brief Function that returns a peer structure from the relative A-CTU position
 * @details This function searches through a linked list of peers
 *          and tries to match a Auxiliary CTU with the relative position.
 *          If the search is inconclusive, it returns NULL.
 * 
 * @param pos The A-CTU pad position
 * @return NULL if no peer is found, peer structure otherwise.
*/
struct peer *Aux_CTU_find(uint16_t pos)
{
    struct peer *peer;

    SLIST_FOREACH(peer, &peers, next) {
        if ((!peer->CRU) && (peer->position == pos)) {
            return peer;
        }
    }

    return NULL;
}

/**
 * @brief Function that checks is a A-CTU position is already taken
 * @details This function searches through a linked list of peers
 *          and tries to match a Auxiliary CTU with the relative position.
 *          If the search is inconclusive, it returns 1.
 * 
 * @param pos The A-CTU pad position
 * @return 0 is position is already taken, 1 otherwise.
*/
static uint8_t AUX_CTU_position_empty(uint16_t pos)
{
    uint8_t empty = 1;
    struct peer *peer;

    SLIST_FOREACH(peer, &peers, next) {
    if ((!peer->CRU) && (peer->position == pos)) {
        empty = 0;
        }
    }

    return empty;
}

/**
 * @brief Function that returns a peer structure with a conn_handle
 * @details This function searches through a linked list of peers
 *          and tries to match a connexion handle with one such
 *          peer. If the search is inconclusive, it returns NULL.
 * 
 * @param conn_handle The peer's connexion handle
 * @return NULL if no peer is found, peer structure otherwise.
*/
struct peer *peer_find(uint16_t conn_handle)
{
    struct peer *peer;

    SLIST_FOREACH(peer, &peers, next) {
        if (peer->conn_handle == conn_handle) {
            return peer;
        }
    }

    return NULL;
}

/**
 * @brief Notifies caller if discovery is done.
 * @details This function will call back the caller once it has
 *          asserted that a peer has been fully added to the
 *          list (fully discovered).
 * 
 * @param peer The peer structure being built
 * @param rc   Return code integer
*/
static void peer_disc_complete(struct peer *peer, int rc)
{
    peer->disc_prev_chr_val = 0;

    /* Notify caller that discovery has completed. */
    if (peer->disc_cb != NULL) {
        peer->disc_cb(peer, rc, peer->disc_cb_arg);
    }
}

/**
 * @brief Function delegated to finding a previous descriptor
 * @details This function will try and find a previously inputed
 *          descriptor.
 * 
 * @param chr A peer characteristic structure
 * @param dsc_handle A descriptor handle
 * 
 * @return Previous descriptor in the list.
*/
static struct peer_dsc *peer_dsc_find_prev(const struct peer_chr *chr, uint16_t dsc_handle)
{
    struct peer_dsc *prev;
    struct peer_dsc *dsc;

    prev = NULL;
    SLIST_FOREACH(dsc, &chr->dscs, next) {
        if (dsc->dsc.handle >= dsc_handle) {
            break;
        }
        prev = dsc;
    }

    return prev;
}

/**
 * @brief Function delegated to finding a descriptor
 * @details This function will try and find a previously inputed
 *          descriptor. Then it will compare the descriptor handles
 *          and determine if it is the correct descriptor to return.
 * 
 * @param chr A peer characteristic structure
 * @param dsc_handle A descriptor handle
 * @param out_prev A descriptor handle
 * 
 * @return A descriptor structure, or NULL is it was not found.
*/
static struct peer_dsc *peer_dsc_find(const struct peer_chr *chr, uint16_t dsc_handle,
              struct peer_dsc **out_prev)
{
    struct peer_dsc *prev;
    struct peer_dsc *dsc;

    prev = peer_dsc_find_prev(chr, dsc_handle);
    if (prev == NULL) {
        dsc = SLIST_FIRST(&chr->dscs);
    } else {
        dsc = SLIST_NEXT(prev, next);
    }

    if (dsc != NULL && dsc->dsc.handle != dsc_handle) {
        dsc = NULL;
    }

    if (out_prev != NULL) {
        *out_prev = prev;
    }
    return dsc;
}

/**
 * @brief Adds a descriptor to a particular characteristic.
 * @details This function will add a new GATT descriptor to an
 *          already existing characteristic value. Starting
 *          from a peer structure, it will try to match a
 *          characteristic value handle with the characteristic
 *          list. Then, if it exists within an already existing
 *          BLE service, the function will add a GATT 
 *          descriptor structure to the correct characteristic
 *          value.
 * 
 * @param peer A peer structure
 * @param char_val_handle A characteristic value handle
 * @param gatt_dsc A GATT descriptor structure
 * 
 * @return 0 on success, BLE_HS error codes otherwise
*/
static int peer_dsc_add(struct peer *peer, uint16_t chr_val_handle,
             const struct ble_gatt_dsc *gatt_dsc)
{
    struct peer_dsc *prev;
    struct peer_dsc *dsc;
    struct peer_svc *svc;
    struct peer_chr *chr;

    svc = peer_svc_find_range(peer, chr_val_handle);
    if (svc == NULL) {
        /* Can't find service for discovered descriptor; this shouldn't
         * happen.
         */
        return BLE_HS_EUNKNOWN;
    }

    chr = peer_chr_find(svc, chr_val_handle, NULL);
    if (chr == NULL) {
        /* Can't find characteristic for discovered descriptor; this shouldn't
         * happen.
         */
        return BLE_HS_EUNKNOWN;
    }

    dsc = peer_dsc_find(chr, gatt_dsc->handle, &prev);
    if (dsc != NULL) {
        /* Descriptor already discovered. */
        return 0;
    }

    dsc = os_memblock_get(&peer_dsc_pool);
    if (dsc == NULL) {
        /* Out of memory. */
        return BLE_HS_ENOMEM;
    }
    memset(dsc, 0, sizeof * dsc);

    dsc->dsc = *gatt_dsc;

    if (prev == NULL) {
        SLIST_INSERT_HEAD(&chr->dscs, dsc, next);
    } else {
        SLIST_NEXT(prev, next) = dsc;
    }

    return 0;
}

/**
 * @brief Allows the discovery of descriptors
 * @details This function discovers all characteristic
 *          descriptors within all BLE services. It begins
 *          by iterating through all services, all
 *          characteristics and then all descriptors.
 *          Whenever such descriptor is discovered, it is
 *          added to the peers' descriptor list.
 * 
 * @param peer A peer structure
*/
static void peer_disc_dscs(struct peer *peer)
{
    struct peer_chr *chr;
    struct peer_svc *svc;
    int rc;

    /* Search through the list of discovered characteristics for the first
     * characteristic that contains undiscovered descriptors.  Then, discover
     * all descriptors belonging to that characteristic.
     */
    SLIST_FOREACH(svc, &peer->svcs, next) {
        SLIST_FOREACH(chr, &svc->chrs, next) {
            if (!chr_is_empty(svc, chr) &&
                    SLIST_EMPTY(&chr->dscs) &&
                    peer->disc_prev_chr_val <= chr->chr.def_handle) {

                rc = ble_gattc_disc_all_dscs(peer->conn_handle,
                                             chr->chr.val_handle,
                                             chr_end_handle(svc, chr),
                                             peer_dsc_disced, peer);
                if (rc != 0) {
                    peer_disc_complete(peer, rc);
                }

                peer->disc_prev_chr_val = chr->chr.val_handle;
                return;
            }
        }
    }

    /* All descriptors discovered. */
    peer_disc_complete(peer, 0);
}

/**
 * @brief Descriptor discovery callback function
 * @details This function runs asynchronously once after a discovery
 *          attempt from peer_disc_dscs function. It evaluates if
 *          no error has occured during the discovery procedure,
 *          adds the descipror to the peer structures' respective
 *          characteristic and asserts if this specific descriptor
 *          is the last descriptor to discover from the
 *          characteristic identified by its value handle.
 * 
 *          Any GATT error will result in termination of the
 *          discovery procedure.
 * 
 * @param conn_handle The peer's connexion handle
 * @param error A GATT protocol compliant return error
 * @param chr_val_handle A characteristic value handle
 * @param dsc A characteristic descriptor
 * @param arg Additional argument
 * 
 * @return 0 on success, GATT error codes otherwise
*/
static int peer_dsc_disced(uint16_t conn_handle, const struct ble_gatt_error *error,
                uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                void *arg)
{
    struct peer *peer;
    int rc;

    peer = arg;

    switch (error->status) {
    case 0:
        rc = peer_dsc_add(peer, chr_val_handle, dsc);
        break;

    case BLE_HS_EDONE:
        /* All descriptors in this characteristic discovered; start discovering
         * descriptors in the next characteristic.
         */
        if (peer->disc_prev_chr_val > 0) {
            peer_disc_dscs(peer);
        }
        rc = 0;
        break;

    default:
        /* Error; abort discovery. */
        rc = error->status;
        break;
    }

    if (rc != 0) {
        /* Error; abort discovery. */
        peer_disc_complete(peer, rc);
    }

    return rc;
}

/**
 * @brief Asserts if a given characteristic handle is the lists' end
 * @details This function determines, from a given service and
 *          characteristic, if said characteristic is the last one
 *          (or end) of the linked list which is used to store them.
 * 
 * @param svc A particular peer supported service structure
 * @param chr A particular peer supported characteristic structure
 * 
 * @return Either a service handle or the next characteristics' defined handle.
*/
uint16_t chr_end_handle(const struct peer_svc *svc, const struct peer_chr *chr)
{
    const struct peer_chr *next_chr;

    next_chr = SLIST_NEXT(chr, next);
    if (next_chr != NULL) {
        return next_chr->chr.def_handle - 1;
    } else {
        return svc->svc.end_handle;
    }
}

/**
 * @brief Asserts if a given characteristic is empty
 * @details This function determines, from a given service and
 *          characteristic, if said characteristic is empty.
 * 
 * @param svc A particular peer supported service structure
 * @param chr A particular peer supported characteristic structure
 * 
 * @return 1 if the characteristic is empty, 0 otherwise.
*/
int chr_is_empty(const struct peer_svc *svc, const struct peer_chr *chr)
{
    return chr_end_handle(svc, chr) <= chr->chr.val_handle;
}


/**
 * @brief Returns the preceeding characteristic from another
 *        characteristic
 * @details This function returns the previously inputed characteristic
 *          in reference to a given characteristic value handle. It
 *          iterates through all peer characteristic structures and once
 *          there is a match on chr_val_handle, the characteristic value
 *          handle that preceeds this one is used to return its
 *          characteristic structure.
 * 
 * @param svc A particular peer supported service structure
 * @param chr_val_handle A particular peer supported characteristic structure
 * 
 * @return A characteristic structure if no problem occurs, NULL otherwise.
*/
static struct peer_chr *peer_chr_find_prev(const struct peer_svc *svc, uint16_t chr_val_handle)
{
    struct peer_chr *prev;
    struct peer_chr *chr;

    prev = NULL;
    SLIST_FOREACH(chr, &svc->chrs, next) {
        if (chr->chr.val_handle >= chr_val_handle) {
            break;
        }

        prev = chr;
    }

    return prev;
}

/**
 * @brief Returns the preceeding characteristic in reference to another
 *        characteristic.
 * @details This function returns the previously inputed characteristic
 *          in reference to a given characteristic value handle. It
 *          iterates through all peer characteristic structures and once
 *          there is a match on chr_val_handle, the characteristic value
 *          handle that preceeds this one is used to return its
 *          characteristic structure.
 * 
 * @param svc A particular peer supported service structure
 * @param chr_val_handle A characteristic value handle
 * @param out_prev Pointer to the preceeding characteristic found
 * 
 * @return A characteristic structure if no problem occurs, NULL otherwise.
*/
static struct peer_chr *peer_chr_find(const struct peer_svc *svc, uint16_t chr_val_handle,
              struct peer_chr **out_prev)
{
    struct peer_chr *prev;
    struct peer_chr *chr;

    prev = peer_chr_find_prev(svc, chr_val_handle);
    if (prev == NULL) {
        chr = SLIST_FIRST(&svc->chrs);
    } else {
        chr = SLIST_NEXT(prev, next);
    }

    if (chr != NULL && chr->chr.val_handle != chr_val_handle) {
        chr = NULL;
    }

    if (out_prev != NULL) {
        *out_prev = prev;
    }
    return chr;
}

/**
 * @brief Deletes a specific peer characteristic
 * @details This function deletes a characteristic and handles the peer 
 *          characteristic linked lists' state. If such characteristic
 *          exists, the function will delete any "embedded" descriptor
 *          from the characteristic. Then, the memory allocated to this
 *          descriptor (and characteristic thereafter) is freed.
 * 
 * @param chr A particular peer supported characteristic structure
*/
static void peer_chr_delete(struct peer_chr *chr)
{
    struct peer_dsc *dsc;

    while ((dsc = SLIST_FIRST(&chr->dscs)) != NULL) {
        SLIST_REMOVE_HEAD(&chr->dscs, next);
        os_memblock_put_from_cb(&peer_dsc_pool, dsc);
    }

    os_memblock_put_from_cb(&peer_chr_pool, chr);
}

/**
 * @brief Adds a specific peer characteristic
 * @details This function adds a BLE characteristic to the overarching
 *          peer characteristic structure. It starts by finding the very
 *          first service structure from a corresponding service handle.
 *          If such a service exists, it makes sure the characteristic
 *          is not already added to the service. Then, memory is
 *          allocated from the characteristic memory pool to contain
 *          the newly defined characteristic.
 * 
 *          All characteristic bit fields and parameters are set into
 *          the characteristic structure from the gatt_chr input parameter.
 * 
 * @param peer A peer structure
 * @param svc_start_handle The handle from which service definition starts
 * @param gatt_chr A GATT defined structure containing all characteristic
 *                 parameters
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/
static int peer_chr_add(struct peer *peer,  uint16_t svc_start_handle,
             const struct ble_gatt_chr *gatt_chr)
{
    struct peer_chr *prev;
    struct peer_chr *chr;
    struct peer_svc *svc;

    svc = peer_svc_find(peer, svc_start_handle, NULL);
    if (svc == NULL) {
        /* Can't find service for discovered characteristic; this shouldn't
         * happen.
         */
        return BLE_HS_EUNKNOWN;
    }

    chr = peer_chr_find(svc, gatt_chr->def_handle, &prev);
    if (chr != NULL) {
        /* Characteristic already discovered. */
        return 0;
    }

    chr = os_memblock_get(&peer_chr_pool);
    if (chr == NULL) {
        /* Out of memory. */
        return BLE_HS_ENOMEM;
    }
    memset(chr, 0, sizeof * chr);

    chr->chr = *gatt_chr;

    if (prev == NULL) {
        SLIST_INSERT_HEAD(&svc->chrs, chr, next);
    } else {
        SLIST_NEXT(prev, next) = chr;
    }

    return 0;
}

/**
 * @brief Characteristic discovery callback function
 * @details This function runs asynchronously once after a discovery
 *          attempt from peer_disc_chrs function. It evaluates if
 *          no error has occured during the discovery procedure,
 *          adds the characteristic to the peer structures' respective
 *          service and asserts if this specific characteristic is the
 *          last characteristic to discover from the service identified
 *          by its value handle.
 * 
 *          All characteristic bit fields and parameters are set into
 *          the characteristic structure from the chr input parameter.
 * 
 * @param conn_handle The peer's connexion handle
 * @param error A GATT protocol compliant return error
 * @param chr A particular peer supported characteristic structure
 * @param arg Additional argument
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/
static int peer_chr_disced(uint16_t conn_handle, const struct ble_gatt_error *error,
                const struct ble_gatt_chr *chr, void *arg)
{
    struct peer *peer;
    int rc;

    peer = arg;

    //ESP_LOGW(TAG, "we also here man");

    switch (error->status) {
    case 0:
        rc = peer_chr_add(peer, peer->cur_svc->svc.start_handle, chr);
        break;

    case BLE_HS_EDONE:
        /* All characteristics in this service discovered; start discovering
         * characteristics in the next service.
         */
        if (peer->disc_prev_chr_val > 0) {
            peer_disc_chrs(peer);
        }
        rc = 0;
        break;

    default:
        rc = error->status;
        break;
    }

    if (rc != 0) {
        /* Error; abort discovery. */
        peer_disc_complete(peer, rc);
    }

    return rc;
}

/**
 * @brief Allows the discovery of characteristics
 * @details This function discovers all service
 *          characteristics. It begins by iterating through
 *          all services and then all characteristics.
 *          Whenever such characteristic is discovered, it is
 *          added to the peers' characteristic list.
 * 
 * @param peer A peer structure
*/
static void peer_disc_chrs(struct peer *peer)
{
    struct peer_svc *svc;
    int rc = 0;
    /* Search through the list of discovered service for the first service that
     * contains undiscovered characteristics.  Then, discover all
     * characteristics belonging to that service.
     */
    SLIST_FOREACH(svc, &peer->svcs, next) {
        if (!peer_svc_is_empty(svc) && SLIST_EMPTY(&svc->chrs)) {
            peer->cur_svc = svc;
            rc = ble_gattc_disc_all_chrs(peer->conn_handle,
                                         svc->svc.start_handle,
                                         svc->svc.end_handle,
                                         peer_chr_disced, peer);
            if (rc != 0)
            {
                //ESP_LOGW(TAG, "End Characteristic discovery");
                peer_disc_complete(peer, rc);
            }
            return;
        }
    }

    /* All characteristics discovered. */
    peer_disc_dscs(peer);
}

/**
 * @brief Asserts if a given service is empty
 * @details This function determines, from a given service if
 *          said service is empty.
 * 
 * @param svc A particular peer supported service structure
 * 
 * @return 1 if the service is empty, 0 otherwise.
*/
int peer_svc_is_empty(const struct peer_svc *svc)
{
    return svc->svc.end_handle <= svc->svc.start_handle;
}

/**
 * @brief Returns the preceeding service in reference to another BLE
 *        service.
 * @details This function returns the previously inputed BLE service
 *          in reference to a given BLE service handle. It iterates
 *          through all peer service structures and once there is a
 *          match on svc_start_handle, the service handle that
 *          preceeds this one is used to return its service structure.
 * 
 * @param peer A particular peer supported service structure
 * @param svc_start_handle A characteristic value handle
 * 
 * @return A characteristic structure if no problem occurs, NULL otherwise.
*/
static struct peer_svc *peer_svc_find_prev(struct peer *peer, uint16_t svc_start_handle)
{
    struct peer_svc *prev;
    struct peer_svc *svc;

    prev = NULL;
    SLIST_FOREACH(svc, &peer->svcs, next) {
        if (svc->svc.start_handle >= svc_start_handle) {
            break;
        }

        prev = svc;
    }

    return prev;
}

/**
 * @brief Returns the preceeding service from another service
 * @details This function returns the previously inputed service in
 *          reference to the very first service handle handle. It
 *          iterates through all peer service structures and once
 *          there is a match on svc_start_handle, the service handle
 *          that preceeds this one is used to return its service
 *          structure.
 * 
 * @param peer A peer structure
 * @param svc_start_handle The very first (start) service handle
 * @param out_prev A pointer to the previous service structure
 * 
 * @return A service structure if no problem occurs, NULL otherwise.
*/
static struct peer_svc *peer_svc_find(struct peer *peer, uint16_t svc_start_handle,
              struct peer_svc **out_prev)
{
    struct peer_svc *prev;
    struct peer_svc *svc;

    prev = peer_svc_find_prev(peer, svc_start_handle);
    if (prev == NULL) {
        svc = SLIST_FIRST(&peer->svcs);
    } else {
        svc = SLIST_NEXT(prev, next);
    }

    if (svc != NULL && svc->svc.start_handle != svc_start_handle) {
        svc = NULL;
    }

    if (out_prev != NULL) {
        *out_prev = prev;
    }
    return svc;
}

/**
 * @brief Returns a service structure from an attribute handle
 * @details This function returns a particular service structure
 *          from a given attribute handle. By iterating through all
 *          services available on a specific peer, the function
 *          then evaluates if the attribute handle is located
 *          within the {start, end} service handle range.
 * 
 * @param peer A peer structure
 * @param attr_handle An attribute handle
 * 
 * @return A service structure if the service is found, NULL otherwise.
*/
static struct peer_svc *peer_svc_find_range(struct peer *peer, uint16_t attr_handle)
{
    struct peer_svc *svc;

    SLIST_FOREACH(svc, &peer->svcs, next) {
        if (svc->svc.start_handle <= attr_handle &&
                svc->svc.end_handle >= attr_handle) {

            return svc;
        }
    }

    return NULL;
}

/**
 * @brief Returns a service structure from a given UUID
 * @details This function returns a particular service structure
 *          from a given UUID. By iterating through all services
 *          available on a specific peer, the function then
 *          evaluates if the input UUID is the same as the current
 *          service UUID (on each iteration).
 * 
 * @param peer A peer structure
 * @param uuid A service UUID
 * 
 * @return A service structure if the service is found, NULL otherwise.
*/
const struct peer_svc *peer_svc_find_uuid(const struct peer *peer, const ble_uuid_t *uuid)
{
    const struct peer_svc *svc;

    SLIST_FOREACH(svc, &peer->svcs, next) {
              
    if (ble_uuid_cmp(&svc->svc.uuid.u, uuid) == 0) {
            //ESP_LOGW(TAG, "WPT SERVICE FOUND");
            return svc;
        }
    }

    return NULL;
}


/**
 * @brief Returns a characteristic structure from a given UUID
 * @details This function returns a particular characteristics structure
 *          from a given UUID. First, the necessary service is found with
 *          the help of an inputed service UUID. Afterwards, the function
 *          iterates through all characteristics of the service that was
 *          found and evaluates if the input UUID is the same as the current
 *          characteristic UUID (on each iteration).
 *          
 * @param peer A peer structure
 * @param svc_uuid A service UUID
 * @param chr_uuid A characteristic UUID
 * 
 * @return A characteristic structure if the characteristic is found,
 *         NULL otherwise.
*/
const struct peer_chr *peer_chr_find_uuid(const struct peer *peer, const ble_uuid_t *svc_uuid,
                   const ble_uuid_t *chr_uuid)
{
    const struct peer_svc *svc;
    const struct peer_chr *chr;
    
    svc = peer_svc_find_uuid(peer, svc_uuid);
    if (svc == NULL) {
        //ESP_LOGW(TAG, "NO SERVICES FOUND");
        return NULL;
    }

    SLIST_FOREACH(chr, &svc->chrs, next) {
        if (ble_uuid_cmp(&chr->chr.uuid.u, chr_uuid) == 0) {
            //ESP_LOGW(TAG, "CHARACTERISTIC FOUND");
            return chr;
        }
    }
    

    return NULL;
}

/**
 * @brief Returns a descriptor structure from a given UUID
 * @details This function returns a particular descriptor structure
 *          from a given UUID. First, the necessary characteristic is found with
 *          the help of an service/characteristic UUID pair. Afterwards,
 *          the function iterates through all decriptor of the characteristic 
 *          that was found and evaluates if the input UUID is the same as the current
 *          descriptor UUID (on each iteration).
 *          
 * @param peer A peer structure
 * @param svc_uuid A service UUID
 * @param chr_uuid A characteristic UUID
 * @param dsc_uuid A descriptor UUID
 * 
 * @return A descriptor structure if the descriptor is found, NULL otherwise.
*/
const struct peer_dsc *peer_dsc_find_uuid(const struct peer *peer, const ble_uuid_t *svc_uuid,
                   const ble_uuid_t *chr_uuid, const ble_uuid_t *dsc_uuid)
{
    const struct peer_chr *chr;
    const struct peer_dsc *dsc;

    chr = peer_chr_find_uuid(peer, svc_uuid, chr_uuid);
    if (chr == NULL) {
        return NULL;
    }

    SLIST_FOREACH(dsc, &chr->dscs, next) {
        if (ble_uuid_cmp(&dsc->dsc.uuid.u, dsc_uuid) == 0) {
            return dsc;
        }
    }

    return NULL;
}

/**
 * @brief Adds a specific peer service
 * @details This function adds a BLE service to the overarching peer
 *          service structure. It starts by determining if said service
 *          already exists within the peer structure. Then, memory is
 *          allocated from the BLE service memory pool to contain the
 *          newly defined service.
 * 
 *          All service bit fields and parameters are set into the
 *          service structure from the gatt_svc input parameter.
 * 
 * @param peer A peer structure
 * @param gatt_svc A GATT defined structure containing all service
 *                 parameters
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/
static int peer_svc_add(struct peer *peer, const struct ble_gatt_svc *gatt_svc)
{
    struct peer_svc *prev;
    struct peer_svc *svc;

    svc = peer_svc_find(peer, gatt_svc->start_handle, &prev);
    if (svc != NULL) {
        /* Service already discovered. */
        return 0;
    }

    svc = os_memblock_get(&peer_svc_pool);
    if (svc == NULL) {
        /* Out of memory. */
        return BLE_HS_ENOMEM;
    }
    memset(svc, 0, sizeof * svc);

    svc->svc = *gatt_svc;
    SLIST_INIT(&svc->chrs);

    if (prev == NULL) {
        SLIST_INSERT_HEAD(&peer->svcs, svc, next);
    } else {
        SLIST_INSERT_AFTER(prev, svc, next);
    }

    return 0;
}

/**
 * @brief Deletes a specific peer service
 * @details This function deletes a service and handles the peer 
 *          service linked lists' state. If such service exists,
 *          the function will delete any "embedded" characteristic
 *          and their respective descriptors. Then, the memory
 *          allocated to this service is freed.
 * 
 * @param svc A particular peer supported service structure
*/
static void peer_svc_delete(struct peer_svc *svc)
{
    struct peer_chr *chr;

    while ((chr = SLIST_FIRST(&svc->chrs)) != NULL) {
        SLIST_REMOVE_HEAD(&svc->chrs, next);
        peer_chr_delete(chr);
    }

    os_memblock_put_from_cb(&peer_svc_pool, svc);
}

/**
 * @brief Service discovery callback function
 * @details This function runs asynchronously once after a discovery
 *          attempt from peer_disc_svcs function. It evaluates if
 *          no error has occured during the discovery procedure,
 *          adds the service to the peer structures' respective
 *          service structure and asserts if this specific BLE service
 *          is the last service to discover from the peer advertisement.
 * 
 *          All service bit fields and parameters are set into the
 *          service structure from the service input parameter.
 * 
 * @param conn_handle The peer's connexion handle
 * @param error A GATT protocol compliant return error
 * @param service A particular peer supported characteristic structure
 * @param arg Additional argument
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/
static int peer_svc_disced(uint16_t conn_handle, const struct ble_gatt_error *error,
                const struct ble_gatt_svc *service, void *arg)
{
    struct peer *peer;
    int rc;

    peer = arg;

    switch (error->status) {
    case 0:
        //ESP_LOGW(TAG, "HERE 1");
        //ESP_LOGW(TAG, "Service: %d", (int)&service->uuid.u128.u);

        rc = peer_svc_add(peer, service);
        break;

    case BLE_HS_EDONE:
        /* All s discovered; start discovering characteristics. */
        //ESP_LOGW(TAG, "HERE 2");
        if (peer->disc_prev_chr_val > 0) {
            peer_disc_chrs(peer);
        }
        rc = 0;
        break;

    default:
        rc = error->status;
        break;
    }

    if (rc != 0) {
        /* Error; abort discovery. */
        peer_disc_complete(peer, rc);
    }

    return rc;
}

/**
 * @brief Peer BLE stack full discovery
 * @details This function tries to discover all possible services,
 *          their characteristics and their descriptors.
 * 
 * @param conn_handle The peer's connexion handle
 * @param disc_cb A callback function which returns to ble_central file.
 * @param arg Additional argument
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/

int peer_disc_all(uint16_t conn_handle, peer_disc_fn *disc_cb, void *arg)
{
    struct peer_svc *svc;
    struct peer *peer;
    int rc;

    peer = peer_find(conn_handle);
    if (peer == NULL) {
        return BLE_HS_ENOTCONN;
    }
    /* Undiscover everything first. */
    while ((svc = SLIST_FIRST(&peer->svcs)) != NULL) {
        SLIST_REMOVE_HEAD(&peer->svcs, next);
        peer_svc_delete(svc); 
    }
    peer->disc_prev_chr_val = 1;
    peer->disc_cb = disc_cb;
    peer->disc_cb_arg = arg;

//* DISC ALL SERVICES

    //rc = ble_gattc_disc_all_svcs(conn_handle, peer_svc_disced, peer);
    rc = ble_gattc_disc_svc_by_uuid(conn_handle, wpt_svc_uuid, peer_svc_disced, peer);
    if (rc != 0) {
        ESP_LOGE(TAG, "eìwups");
        return rc;
    }

    return 0;
}

/**
 * @brief Peer deletion function
 * @details This function tries to delete a specific peer structure. It
 *          first asserts if such peer exists by finding it among all
 *          peer structures. It then systematicallly deletes services,
 *          characteristics and their descriptors. Once all fo that memory
 *          has been freed, the mamory allocated for the peer structure is
 *          itself freed. The total number of peers is then reduced and if
 *          no errors occure, the program return to normal operation with
 *          fewer peers (or perhaps no remaining peers).
 * 
 * @param conn_handle The peer's connexion handle
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/
int peer_delete(uint16_t conn_handle)
{
    struct peer_svc *svc;
    struct peer *peer;
    int rc;

    peer = peer_find(conn_handle);

    if(peer->CRU) {
        NUM_CRU--;
    } else {
        NUM_AUX_CTU--;
    }

    ESP_LOGW(TAG, "Deleting peer with conn_handle=%d; %d peers remaining", 
                    conn_handle, NUM_CRU + NUM_AUX_CTU);

    if (peer == NULL) {
        return BLE_HS_ENOTCONN;
    }  

    if (peer->sem_handle != NULL)
    {
        vSemaphoreDelete(peer->sem_handle);
    }

    if (peer->task_handle != NULL)
    {
        esp_task_wdt_delete(peer->task_handle);
        vTaskDelete(peer->task_handle);
    }

    SLIST_REMOVE(&peers, peer, peer, next);

    while ((svc = SLIST_FIRST(&peer->svcs)) != NULL) {
        SLIST_REMOVE_HEAD(&peer->svcs, next);
        peer_svc_delete(svc);
    }

    rc = os_memblock_put_from_cb(&peer_pool, peer);
    if (rc != 0) {
        return BLE_HS_EOS;
    }

    return 0;
}

/**
 * @brief Peer addition function
 * @details This function tries to add a new peer structure to the main
 *          linked list. It first asserts if such peer exists. If it is
 *          the case, then an error code is returned. Memory is then
 *          allocated in the memory pool, and it takes its place in
 *          the linked list of peer structures. The peers' connection handle
 *          is attributed and the total number of peers is incremented.
 * 
 * @param conn_handle The peer's connexion handle
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/
int peer_add(uint16_t conn_handle, bool CRU)
{
    struct peer *peer;

    if ((MYNEWT_VAL(BLE_MAX_CONNECTIONS) > NUM_CRU + NUM_AUX_CTU))
    {
        /* Make sure the connection handle is unique. */
        peer = peer_find(conn_handle);
        if (peer != NULL) {
            return BLE_HS_EALREADY;
        }

        peer = os_memblock_get(&peer_pool);
        if (peer == NULL) {
            /* Out of memory. */
            return BLE_HS_ENOMEM;
        }

        memset(peer, 0, sizeof * peer);
        
        if(CRU) {
            NUM_CRU++;
        } else {
            NUM_AUX_CTU++;
            //todo: use MAC address for setting the position
            //add position of the pad
            //check nobody else is in the same position before declaring it
         /*   for(int8_t i = 1; i < 5; i++)
            {
                if(AUX_CTU_position_empty(i))
                {
                    peer->position = i;
                    ESP_LOGI(TAG, "AUX-CTU POSITION = %d", peer->position);
                    break;  
                } 
            }  
           */           
        }
        peer->conn_handle = conn_handle;
        peer->CRU = CRU;

        SLIST_INSERT_HEAD(&peers, peer, next);

        return 0;
    }
    return BLE_HS_ENOENT;
}

/**
 * @brief Peer memory deallocation function
 * @details This function cleans any information from all
 *          allocated peer memory pools. No linked list shall
 *          remain after this function is called.
*/
static void peer_free_mem(void)
{
    free(peer_mem);
    peer_mem = NULL;

    free(peer_svc_mem);
    peer_svc_mem = NULL;

    free(peer_chr_mem);
    peer_chr_mem = NULL;

    free(peer_dsc_mem);
    peer_dsc_mem = NULL;
}

/**
 * @brief Peer structure initialization function
 * @details This function initializes the memory pool for all
 *          peer components, such as BLE services,
 *          characteristics, desciptors and any other auxiliary
 *          information related to peers.
 * 
 * @param max_peers The maximum number of peers allowed
 * @param max_svcs The maximum number of services allowed
 * @param max_chrs The maximum number of characteristics allowed
 * @param max_dscs The maximum number of descriptors allowed
 * 
 * @return 0 on success, BLE_HS error code otherwise.
*/
int peer_init(int max_peers, int max_svcs, int max_chrs, int max_dscs)
{
    int rc;

    /* Free memory first in case this function gets called more than once. */
    peer_free_mem();

    peer_mem = malloc(OS_MEMPOOL_BYTES(max_peers, sizeof (struct peer)));
    if (peer_mem == NULL) {
        rc = BLE_HS_ENOMEM;
        goto err;
    }

    rc = os_mempool_init(&peer_pool, max_peers,
                         sizeof (struct peer), peer_mem,
                         "peer_pool");
    if (rc != 0) {
        rc = BLE_HS_EOS;
        goto err;
    }

    peer_svc_mem = malloc(OS_MEMPOOL_BYTES(max_svcs, sizeof (struct peer_svc)));
    if (peer_svc_mem == NULL) {
        rc = BLE_HS_ENOMEM;
        goto err;
    }

    rc = os_mempool_init(&peer_svc_pool, max_svcs,
                         sizeof (struct peer_svc), peer_svc_mem,
                         "peer_svc_pool");
    if (rc != 0) {
        rc = BLE_HS_EOS;
        goto err;
    }

    peer_chr_mem = malloc(OS_MEMPOOL_BYTES(max_chrs, sizeof (struct peer_chr)));
    if (peer_chr_mem == NULL) {
        rc = BLE_HS_ENOMEM;
        goto err;
    }

    rc = os_mempool_init(&peer_chr_pool, max_chrs,
                         sizeof (struct peer_chr), peer_chr_mem,
                         "peer_chr_pool");
    if (rc != 0) {
        rc = BLE_HS_EOS;
        goto err;
    }

    peer_dsc_mem = malloc(OS_MEMPOOL_BYTES(max_dscs, sizeof (struct peer_dsc)));
    if (peer_dsc_mem == NULL) {
        rc = BLE_HS_ENOMEM;
        goto err;
    }

    rc = os_mempool_init(&peer_dsc_pool, max_dscs,
                         sizeof (struct peer_dsc), peer_dsc_mem,
                         "peer_dsc_pool");
    if (rc != 0) {
        rc = BLE_HS_EOS;
        goto err;
    }

    return 0;

err:
    peer_free_mem();
    return rc;
}
