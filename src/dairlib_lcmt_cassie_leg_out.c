// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "dairlib_lcmt_cassie_leg_out.h"

static int __dairlib_lcmt_cassie_leg_out_hash_computed;
static uint64_t __dairlib_lcmt_cassie_leg_out_hash;

uint64_t __dairlib_lcmt_cassie_leg_out_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __dairlib_lcmt_cassie_leg_out_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = __dairlib_lcmt_cassie_leg_out_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x96f894bbc015d472LL
         + __dairlib_lcmt_elmo_out_hash_recursive(&cp)
         + __dairlib_lcmt_elmo_out_hash_recursive(&cp)
         + __dairlib_lcmt_elmo_out_hash_recursive(&cp)
         + __dairlib_lcmt_elmo_out_hash_recursive(&cp)
         + __dairlib_lcmt_elmo_out_hash_recursive(&cp)
         + __dairlib_lcmt_cassie_joint_out_hash_recursive(&cp)
         + __dairlib_lcmt_cassie_joint_out_hash_recursive(&cp)
         + __dairlib_lcmt_cassie_joint_out_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int16_t_hash_recursive(&cp)
         + __boolean_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __dairlib_lcmt_cassie_leg_out_get_hash(void)
{
    if (!__dairlib_lcmt_cassie_leg_out_hash_computed) {
        __dairlib_lcmt_cassie_leg_out_hash = (int64_t)__dairlib_lcmt_cassie_leg_out_hash_recursive(NULL);
        __dairlib_lcmt_cassie_leg_out_hash_computed = 1;
    }

    return __dairlib_lcmt_cassie_leg_out_hash;
}

int __dairlib_lcmt_cassie_leg_out_encode_array(void *buf, int offset, int maxlen, const dairlib_lcmt_cassie_leg_out *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __dairlib_lcmt_elmo_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].hipRollDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].hipYawDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].hipPitchDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].kneeDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].footDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_cassie_joint_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].shinJoint), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_cassie_joint_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].tarsusJoint), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_cassie_joint_out_encode_array(buf, offset + pos, maxlen - pos, &(p[element].footJoint), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].medullaCounter), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].medullaCpuLoad), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &(p[element].reedSwitchState), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int dairlib_lcmt_cassie_leg_out_encode(void *buf, int offset, int maxlen, const dairlib_lcmt_cassie_leg_out *p)
{
    int pos = 0, thislen;
    int64_t hash = __dairlib_lcmt_cassie_leg_out_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __dairlib_lcmt_cassie_leg_out_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __dairlib_lcmt_cassie_leg_out_encoded_array_size(const dairlib_lcmt_cassie_leg_out *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __dairlib_lcmt_elmo_out_encoded_array_size(&(p[element].hipRollDrive), 1);

        size += __dairlib_lcmt_elmo_out_encoded_array_size(&(p[element].hipYawDrive), 1);

        size += __dairlib_lcmt_elmo_out_encoded_array_size(&(p[element].hipPitchDrive), 1);

        size += __dairlib_lcmt_elmo_out_encoded_array_size(&(p[element].kneeDrive), 1);

        size += __dairlib_lcmt_elmo_out_encoded_array_size(&(p[element].footDrive), 1);

        size += __dairlib_lcmt_cassie_joint_out_encoded_array_size(&(p[element].shinJoint), 1);

        size += __dairlib_lcmt_cassie_joint_out_encoded_array_size(&(p[element].tarsusJoint), 1);

        size += __dairlib_lcmt_cassie_joint_out_encoded_array_size(&(p[element].footJoint), 1);

        size += __int8_t_encoded_array_size(&(p[element].medullaCounter), 1);

        size += __int16_t_encoded_array_size(&(p[element].medullaCpuLoad), 1);

        size += __boolean_encoded_array_size(&(p[element].reedSwitchState), 1);

    }
    return size;
}

int dairlib_lcmt_cassie_leg_out_encoded_size(const dairlib_lcmt_cassie_leg_out *p)
{
    return 8 + __dairlib_lcmt_cassie_leg_out_encoded_array_size(p, 1);
}

int __dairlib_lcmt_cassie_leg_out_decode_array(const void *buf, int offset, int maxlen, dairlib_lcmt_cassie_leg_out *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __dairlib_lcmt_elmo_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].hipRollDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].hipYawDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].hipPitchDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].kneeDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_elmo_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].footDrive), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_cassie_joint_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].shinJoint), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_cassie_joint_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].tarsusJoint), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dairlib_lcmt_cassie_joint_out_decode_array(buf, offset + pos, maxlen - pos, &(p[element].footJoint), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].medullaCounter), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].medullaCpuLoad), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &(p[element].reedSwitchState), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __dairlib_lcmt_cassie_leg_out_decode_array_cleanup(dairlib_lcmt_cassie_leg_out *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __dairlib_lcmt_elmo_out_decode_array_cleanup(&(p[element].hipRollDrive), 1);

        __dairlib_lcmt_elmo_out_decode_array_cleanup(&(p[element].hipYawDrive), 1);

        __dairlib_lcmt_elmo_out_decode_array_cleanup(&(p[element].hipPitchDrive), 1);

        __dairlib_lcmt_elmo_out_decode_array_cleanup(&(p[element].kneeDrive), 1);

        __dairlib_lcmt_elmo_out_decode_array_cleanup(&(p[element].footDrive), 1);

        __dairlib_lcmt_cassie_joint_out_decode_array_cleanup(&(p[element].shinJoint), 1);

        __dairlib_lcmt_cassie_joint_out_decode_array_cleanup(&(p[element].tarsusJoint), 1);

        __dairlib_lcmt_cassie_joint_out_decode_array_cleanup(&(p[element].footJoint), 1);

        __int8_t_decode_array_cleanup(&(p[element].medullaCounter), 1);

        __int16_t_decode_array_cleanup(&(p[element].medullaCpuLoad), 1);

        __boolean_decode_array_cleanup(&(p[element].reedSwitchState), 1);

    }
    return 0;
}

int dairlib_lcmt_cassie_leg_out_decode(const void *buf, int offset, int maxlen, dairlib_lcmt_cassie_leg_out *p)
{
    int pos = 0, thislen;
    int64_t hash = __dairlib_lcmt_cassie_leg_out_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __dairlib_lcmt_cassie_leg_out_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int dairlib_lcmt_cassie_leg_out_decode_cleanup(dairlib_lcmt_cassie_leg_out *p)
{
    return __dairlib_lcmt_cassie_leg_out_decode_array_cleanup(p, 1);
}

int __dairlib_lcmt_cassie_leg_out_clone_array(const dairlib_lcmt_cassie_leg_out *p, dairlib_lcmt_cassie_leg_out *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __dairlib_lcmt_elmo_out_clone_array(&(p[element].hipRollDrive), &(q[element].hipRollDrive), 1);

        __dairlib_lcmt_elmo_out_clone_array(&(p[element].hipYawDrive), &(q[element].hipYawDrive), 1);

        __dairlib_lcmt_elmo_out_clone_array(&(p[element].hipPitchDrive), &(q[element].hipPitchDrive), 1);

        __dairlib_lcmt_elmo_out_clone_array(&(p[element].kneeDrive), &(q[element].kneeDrive), 1);

        __dairlib_lcmt_elmo_out_clone_array(&(p[element].footDrive), &(q[element].footDrive), 1);

        __dairlib_lcmt_cassie_joint_out_clone_array(&(p[element].shinJoint), &(q[element].shinJoint), 1);

        __dairlib_lcmt_cassie_joint_out_clone_array(&(p[element].tarsusJoint), &(q[element].tarsusJoint), 1);

        __dairlib_lcmt_cassie_joint_out_clone_array(&(p[element].footJoint), &(q[element].footJoint), 1);

        __int8_t_clone_array(&(p[element].medullaCounter), &(q[element].medullaCounter), 1);

        __int16_t_clone_array(&(p[element].medullaCpuLoad), &(q[element].medullaCpuLoad), 1);

        __boolean_clone_array(&(p[element].reedSwitchState), &(q[element].reedSwitchState), 1);

    }
    return 0;
}

dairlib_lcmt_cassie_leg_out *dairlib_lcmt_cassie_leg_out_copy(const dairlib_lcmt_cassie_leg_out *p)
{
    dairlib_lcmt_cassie_leg_out *q = (dairlib_lcmt_cassie_leg_out*) malloc(sizeof(dairlib_lcmt_cassie_leg_out));
    __dairlib_lcmt_cassie_leg_out_clone_array(p, q, 1);
    return q;
}

void dairlib_lcmt_cassie_leg_out_destroy(dairlib_lcmt_cassie_leg_out *p)
{
    __dairlib_lcmt_cassie_leg_out_decode_array_cleanup(p, 1);
    free(p);
}

int dairlib_lcmt_cassie_leg_out_publish(lcm_t *lc, const char *channel, const dairlib_lcmt_cassie_leg_out *p)
{
      int max_data_size = dairlib_lcmt_cassie_leg_out_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = dairlib_lcmt_cassie_leg_out_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _dairlib_lcmt_cassie_leg_out_subscription_t {
    dairlib_lcmt_cassie_leg_out_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void dairlib_lcmt_cassie_leg_out_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    dairlib_lcmt_cassie_leg_out p;
    memset(&p, 0, sizeof(dairlib_lcmt_cassie_leg_out));
    status = dairlib_lcmt_cassie_leg_out_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding dairlib_lcmt_cassie_leg_out!!!\n", status);
        return;
    }

    dairlib_lcmt_cassie_leg_out_subscription_t *h = (dairlib_lcmt_cassie_leg_out_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    dairlib_lcmt_cassie_leg_out_decode_cleanup (&p);
}

dairlib_lcmt_cassie_leg_out_subscription_t* dairlib_lcmt_cassie_leg_out_subscribe (lcm_t *lcm,
                    const char *channel,
                    dairlib_lcmt_cassie_leg_out_handler_t f, void *userdata)
{
    dairlib_lcmt_cassie_leg_out_subscription_t *n = (dairlib_lcmt_cassie_leg_out_subscription_t*)
                       malloc(sizeof(dairlib_lcmt_cassie_leg_out_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 dairlib_lcmt_cassie_leg_out_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg dairlib_lcmt_cassie_leg_out LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int dairlib_lcmt_cassie_leg_out_subscription_set_queue_capacity (dairlib_lcmt_cassie_leg_out_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int dairlib_lcmt_cassie_leg_out_unsubscribe(lcm_t *lcm, dairlib_lcmt_cassie_leg_out_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe dairlib_lcmt_cassie_leg_out_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

