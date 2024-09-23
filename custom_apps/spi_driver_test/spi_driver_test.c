// /****************************************************************************
//  * custom_apps/custom_hello/custom_hello.c
//  *
//  * Licensed to the Apache Software Foundation (ASF) under one or more
//  * contributor license agreements.  See the NOTICE file distributed with
//  * this work for additional information regarding copyright ownership.  The
//  * ASF licenses this file to you under the Apache License, Version 2.0 (the
//  * "License"); you may not use this file except in compliance with the
//  * License.  You may obtain a copy of the License at
//  *
//  *   http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
//  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
//  * License for the specific language governing permissions and limitations
//  * under the License.
//  *
//  ****************************************************************************/

// /****************************************************************************
//  * Included Files
//  ****************************************************************************/

// #include <nuttx/config.h>
// #include <stdio.h>
// #include <assert.h>
// #include <fcntl.h>
// #include <time.h>
// #include <poll.h>
// #include <unistd.h>
// #include <sys/ioctl.h>

// #include <nuttx/sensors/sensor.h>
// #include <nuttx/sensors/lis3mdl.h>

// #include "spi_driver_test.h"

// #define NB_LOWERHALFS 1



// struct data
// {
//   void *data_struct;
//   uint16_t data_size;
// };


// // #define IOCTL_MODE 1
// // #define READ_MODE   1

// // #define MAX_CHANNELS 12

// /****************************************************************************
//  * spi_driver_test_main
//  ****************************************************************************/

// int main(int argc, FAR char *argv[])
// {
//   int mag_fd;
//   uint16_t seconds;
//   int ret;

//   struct sensor_mag mag;
//   struct sensor_mag mag_sub;

//   int mag_afd, mag_sfd;

//   printf("SPI device LIS3MDL uorb test, World!\n");

//   /* Open SPI device driver */
//   mag_fd = open("/dev/uorb/sensor_mag0", O_RDONLY | O_NONBLOCK);
//   if (mag_fd < 0)
//   {
//     printf("Failed to open mag sensor\n");
//     return -1;
//   }

//   struct pollfd pfds[] = {
//       {.fd = mag_fd, .events = POLLIN}};

//   struct data sensor_data[] = {
//       {.data_struct = &mag, .data_size = sizeof(struct sensor_mag)}};

//   seconds = 5*3;

//   while (seconds > 0)
//   {
//     ret = poll(pfds, NB_LOWERHALFS, -1);
//     if (ret < 0)
//     {
//       perror("Could not poll sensor\n");
//       return ret;
//     }

//     for (int i = 0; i < NB_LOWERHALFS; i++)
//     {
//       if (pfds[i].revents & POLLIN)
//       {
//         ret = read(pfds[i].fd, sensor_data[i].data_struct,
//                    sensor_data[i].data_size);

//         if (ret != sensor_data[i].data_size)
//         {
//           perror("Could not read from sub-sensor.");
//           return ret;
//         }
//       }
//     }
//     seconds -= 3;
//   }

//   printf("Timestamp = %lli\n"
//          "Temperature [c] = %f\n"
//          "mag x axis = %f\n"
//          "mag y axis = %f\n"
//          "mag z axis = %f\n",
//          mag.timestamp, mag.temperature, mag.x, mag.y, mag.z);

//   close(mag_fd);

//   // mag_afd = orb_advertise(ORB_ID(sensor_mag),&mag);
//   // if (mag_afd < 0)
//   // {
//   //   printf("advertise failed: %d\n", errno);
//   // }

//   // orb_publish(ORB_ID(sensor_mag), mag_afd, &mag);
  

//   // mag_sfd = orb_subscribe(ORB_ID(sensor_mag));
//   // if (mag_sfd < 0)
//   // {
//   //   printf("subscribe failed: %d\n", errno);
//   // }
  
//   // if (OK != orb_copy(ORB_ID(sensor_mag), mag_sfd, &mag_sub))
//   // {
//   //   printf("copy failed: %d\n", errno);
//   // }
  
//   // if(mag_sub.timestamp != mag.timestamp)
//   // {
//   //   printf("mismatch adv val: %lli subb val: %lli\n", mag.timestamp, mag_sub.timestamp);
//   // } 

//   return 0;
// }

/*             rs.c        */
/* This program is an encoder/decoder for Reed-Solomon codes. Encoding is in
   systematic form, decoding via the Berlekamp iterative algorithm.
   In the present form , the constants mm, nn, tt, and kk=nn-2tt must be
   specified  (the double letters are used simply to avoid clashes with
   other n,k,t used in other programs into which this was incorporated!)
   Also, the irreducible polynomial used to generate GF(2**mm) must also be
   entered -- these can be found in Lin and Costello, and also Clark and Cain.

   The representation of the elements of GF(2**m) is either in index form,
   where the number is the power of the primitive element alpha, which is
   convenient for multiplication (add the powers modulo 2**m-1) or in
   polynomial form, where the bits represent the coefficients of the
   polynomial representation of the number, which is the most convenient form
   for addition.  The two forms are swapped between via lookup tables.
   This leads to fairly messy looking expressions, but unfortunately, there
   is no easy alternative when working with Galois arithmetic.

   The code is not written in the most elegant way, but to the best
   of my knowledge, (no absolute guarantees!), it works.
   However, when including it into a simulation program, you may want to do
   some conversion of global variables (used here because I am lazy!) to
   local variables where appropriate, and passing parameters (eg array
   addresses) to the functions  may be a sensible move to reduce the number
   of global variables and thus decrease the chance of a bug being introduced.

   This program does not handle erasures at present, but should not be hard
   to adapt to do this, as it is just an adjustment to the Berlekamp-Massey
   algorithm. It also does not attempt to decode past the BCH bound -- see
   Blahut "Theory and practice of error control codes" for how to do this.

              Simon Rockliff, University of Adelaide   21/9/89

   26/6/91 Slight modifications to remove a compiler dependent bug which hadn't
           previously surfaced. A few extra comments added for clarity.
           Appears to all work fine, ready for posting to net!

                  Notice
                 --------
   This program may be freely modified and/or given to whoever wants it.
   A condition of such distribution is that the author's contribution be
   acknowledged by his name being left in the comments heading the program,
   however no responsibility is accepted for any financial or other loss which
   may result from some unforseen errors or malfunctioning of the program
   during use.
                                 Simon Rockliff, 26th June 1991
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define mm  8           /* RS code over GF(2**4) - change to suit */
#define nn  255         /* nn=2**mm -1   length of codeword */
#define tt  8           /* number of errors that can be corrected */
#define kk  239         /* kk = nn-2*tt  */

static int pp [mm+1] = { 1, 0, 1, 1, 1, 0, 0, 0, 1} ; /* specify irreducible polynomial coeffts */
static int alpha_to [nn+1], index_of [nn+1], gg [nn-kk+1] ;
static int recd [nn], data [kk], bb [nn-kk] ;
static inited = 0;

static void generate_gf()
/* generate GF(2**mm) from the irreducible polynomial p(X) in pp[0]..pp[mm]
   lookup tables:  index->polynomial form   alpha_to[] contains j=alpha**i;
                   polynomial form -> index form  index_of[j=alpha**i] = i
   alpha=2 is the primitive element of GF(2**mm)
*/
 {
   register int i, mask ;

  mask = 1 ;
  alpha_to[mm] = 0 ;
  for (i=0; i<mm; i++)
   { alpha_to[i] = mask ;
     index_of[alpha_to[i]] = i ;
     if (pp[i]!=0)
       alpha_to[mm] ^= mask ;
     mask <<= 1 ;
   }
  index_of[alpha_to[mm]] = mm ;
  mask >>= 1 ;
  for (i=mm+1; i<nn; i++)
   { if (alpha_to[i-1] >= mask)
        alpha_to[i] = alpha_to[mm] ^ ((alpha_to[i-1]^mask)<<1) ;
     else alpha_to[i] = alpha_to[i-1]<<1 ;
     index_of[alpha_to[i]] = i ;
   }
  index_of[0] = -1 ;
 }


static void gen_poly()
/* Obtain the generator polynomial of the tt-error correcting, length
  nn=(2**mm -1) Reed Solomon code  from the product of (X+alpha**i), i=1..2*tt
*/
 {
   register int i,j ;

   gg[0] = 2 ;    /* primitive element alpha = 2  for GF(2**mm)  */
   gg[1] = 1 ;    /* g(x) = (X+alpha) initially */
   for (i=2; i<=nn-kk; i++)
    { gg[i] = 1 ;
      for (j=i-1; j>0; j--)
        if (gg[j] != 0)  gg[j] = gg[j-1]^ alpha_to[(index_of[gg[j]]+i)%nn] ;
        else gg[j] = gg[j-1] ;
      gg[0] = alpha_to[(index_of[gg[0]]+i)%nn] ;     /* gg[0] can never be zero */
    }
   /* convert gg[] to index form for quicker encoding */
   for (i=0; i<=nn-kk; i++)  gg[i] = index_of[gg[i]] ;
 }


static void encode_rs() {
   register int i,j ;
   int feedback ;

   for (i=0; i<nn-kk; i++)   bb[i] = 0 ;
   for (i=kk-1; i>=0; i--) {
    feedback = index_of[data[i]^bb[nn-kk-1]] ;
    if (feedback != -1) {
        for (j=nn-kk-1; j>0; j--)
          if (gg[j] != -1)
            bb[j] = bb[j-1]^alpha_to[(gg[j]+feedback)%nn] ;
          else
            bb[j] = bb[j-1] ;
        bb[0] = alpha_to[(gg[0]+feedback)%nn] ;
    } else {
        for (j=nn-kk-1; j>0; j--)
          bb[j] = bb[j-1] ;
        bb[0] = 0 ;
    }
   }
   printf("Encoded parity: ");
   for (i = 0; i < nn-kk; i++) {
       printf("%d ", bb[i]);
   }
   printf("\n");
}
static void decode_rs() ;
void rsdec_204(unsigned char* data_out, unsigned char* data_in) {
    int i;

    if (!inited) {
        // Generate the Galois Field GF(2**mm)
        generate_gf();
        // Compute the generator polynomial for this RS code
        gen_poly();
        inited = 1;
    }

    // Put the transmitted codeword, made up of data plus parity, in recd[]
    // parity
    for (i = 0; i < 204 - 188; ++i) {
        recd[i] = data_in[188 + i];
    }
    // zeroes
    for (i = 0; i < 255 - 204; ++i) {
        recd[204 - 188 + i] = 0;
    }
    // data
    for (i = 0; i < 188; ++i) {
        recd[255 - 188 + i] = data_in[i];
    }

    for (i = 0; i < nn; i++) {
        recd[i] = index_of[recd[i]]; // put recd[i] into index form
    }

    // decode recv[]
    decode_rs(); // recd[] is returned in polynomial form

    for (i = 0; i < 188; ++i) {
        data_out[i] = recd[255 - 188 + i];
    }
}


static void decode_rs() {
   register int i,j,u,q ;
   int elp[nn-kk+2][nn-kk], d[nn-kk+2], l[nn-kk+2], u_lu[nn-kk+2], s[nn-kk+1] ;
   int count=0, syn_error=0, root[tt], loc[tt], z[tt+1], err[nn], reg[tt+1] ;

/* first form the syndromes */
   for (i=1; i<=nn-kk; i++) {
      s[i] = 0 ;
      for (j=0; j<nn; j++)
        if (recd[j]!=-1)
          s[i] ^= alpha_to[(recd[j]+i*j)%nn] ;      /* recd[j] in index form */
      if (s[i]!=0)  syn_error=1 ;        /* set flag if non-zero syndrome => error */
      s[i] = index_of[s[i]] ;
   }
   printf("Syndromes: ");
   for (i = 1; i <= nn-kk; i++) {
       printf("%d ", s[i]);
   }
   printf("\n");

   if (syn_error) {
      d[0] = 0 ; d[1] = s[1] ;
      elp[0][0] = 0 ; elp[1][0] = 1 ;
      for (i=1; i<nn-kk; i++) {
          elp[0][i] = -1 ; elp[1][i] = 0 ;
      }
      l[0] = 0 ; l[1] = 0 ;
      u_lu[0] = -1 ; u_lu[1] = 0 ;
      u = 0 ;

      do {
        u++ ;
        if (d[u]==-1) {
          l[u+1] = l[u] ;
          for (i=0; i<=l[u]; i++) {
             elp[u+1][i] = elp[u][i] ;
             elp[u][i] = index_of[elp[u][i]] ;
          }
        } else {
          q = u-1 ;
          while ((d[q]==-1) && (q>0)) q-- ;
          if (q>0) {
             j=q ;
             do {
               j-- ;
               if ((d[j]!=-1) && (u_lu[q]<u_lu[j]))
                 q = j ;
             } while (j>0) ;
          }
          if (l[u]>l[q]+u-q)  l[u+1] = l[u] ;
          else  l[u+1] = l[q]+u-q ;

          for (i=0; i<nn-kk; i++)    elp[u+1][i] = 0 ;
          for (i=0; i<=l[q]; i++)
            if (elp[q][i]!=-1)
              elp[u+1][i+u-q] = alpha_to[(d[u]+nn-d[q]+elp[q][i])%nn] ;
          for (i=0; i<=l[u]; i++) {
            elp[u+1][i] ^= elp[u][i] ;
            elp[u][i] = index_of[elp[u][i]] ;
          }
        }
        u_lu[u+1] = u-l[u+1] ;

        if (u<nn-kk) {
            if (s[u+1]!=-1)
                   d[u+1] = alpha_to[s[u+1]] ;
            else
              d[u+1] = 0 ;
            for (i=1; i<=l[u+1]; i++)
              if ((s[u+1-i]!=-1) && (elp[u+1][i]!=0))
                d[u+1] ^= alpha_to[(s[u+1-i]+index_of[elp[u+1][i]])%nn] ;
            d[u+1] = index_of[d[u+1]] ;
          }
      } while ((u<nn-kk) && (l[u+1]<=tt)) ;

      u++ ;
      if (l[u]<=tt) {
         for (i=0; i<=l[u]; i++)   elp[u][i] = index_of[elp[u][i]] ;

         for (i=1; i<=l[u]; i++)
           reg[i] = elp[u][i] ;
         count = 0 ;
         for (i=1; i<=nn; i++) {
             q = 1 ;
             for (j=1; j<=l[u]; j++)
              if (reg[j]!=-1) {
                reg[j] = (reg[j]+j)%nn ;
                q ^= alpha_to[reg[j]] ;
              }
             if (!q) {
              root[count] = i;
              loc[count] = nn-i ;
              count++ ;
             }
         }
         if (count==l[u]) {
           for (i=1; i<=l[u]; i++) {
            if ((s[i]!=-1) && (elp[u][i]!=-1))
                 z[i] = alpha_to[s[i]] ^ alpha_to[elp[u][i]] ;
            else if ((s[i]!=-1) && (elp[u][i]==-1))
                  z[i] = alpha_to[s[i]] ;
               else if ((s[i]==-1) && (elp[u][i]!=-1))
                    z[i] = alpha_to[elp[u][i]] ;
                else
                  z[i] = 0 ;
            for (j=1; j<i; j++)
              if ((s[j]!=-1) && (elp[u][i-j]!=-1))
                 z[i] ^= alpha_to[(elp[u][i-j] + s[j])%nn] ;
            z[i] = index_of[z[i]] ;
           }

           for (i=0; i<nn; i++) {
             err[i] = 0 ;
             if (recd[i]!=-1)
               recd[i] = alpha_to[recd[i]] ;
             else  recd[i] = 0 ;
           }
           for (i=0; i<l[u]; i++) {
            err[loc[i]] = 1;
            for (j=1; j<=l[u]; j++)
              if (z[j]!=-1)
                err[loc[i]] ^= alpha_to[(z[j]+j*root[i])%nn] ;
            if (err[loc[i]]!=0) {
               err[loc[i]] = index_of[err[loc[i]]] ;
               q = 0 ;
               for (j=0; j<l[u]; j++)
                 if (j!=i)
                   q += index_of[1^alpha_to[(loc[j]+root[i])%nn]] ;
               q = q % nn ;
               err[loc[i]] = alpha_to[(err[loc[i]]-q+nn)%nn] ;
               recd[loc[i]] ^= err[loc[i]] ;
             }
           }
          } else {
            for (i=0; i<nn; i++)
               if (recd[i]!=-1)
                 recd[i] = alpha_to[recd[i]] ;
               else  recd[i] = 0 ;
          }
       } else {
          for (i=0; i<nn; i++)
            if (recd[i]!=-1)
              recd[i] = alpha_to[recd[i]] ;
            else  recd[i] = 0 ;
       }
    } else {
      for (i=0; i<nn; i++)
         if (recd[i]!=-1)
           recd[i] = alpha_to[recd[i]] ;
         else  recd[i] = 0 ;
    }
}


void rsenc_204(unsigned char* data_out, unsigned char* data_in)
{
  int i;

  if (!inited) {
    /* generate the Galois Field GF(2**mm) */
    generate_gf();
    /* compute the generator polynomial for this RS code */
    gen_poly();

    inited = 1;
  }

  /* zeroes */
  for (i=0; i<255-204; ++i) {
    data[i] = 0;
  }
  /* data */
  for (i=0; i<188; ++i) {
    data[255-204 + i] = data_in[i];
  }
  
  encode_rs();

  for (i=0; i<188; ++i) {
    data_out[i] = data_in[i];
  }
  for (i=0; i<204-188; ++i) {
    data_out[188+i] = bb[i];
  }
  
}
int main(void) {
  unsigned char rs_in[204], rs_out[204];
  int i, j, k;

#ifdef SMALL_PROBLEM_SIZE
#define LENGTH 15000
#else
#define LENGTH 150000
#endif

#ifdef __MINGW32__
#define random() rand()
#endif

//   for (i=0; i<LENGTH; ++i)
   {
    /* Generate random data */
    for (j=0; j<188; ++j) {
      rs_in[j] = (random() & 0xFF);
    }
    rsenc_204(rs_out, rs_in);
    printf("Original data: ");
    for (j = 0; j < 188; j++) {
        printf("%02X ", rs_in[j]);
    }
    printf("\n");

    printf("Encoded data: ");
    for (j = 0; j < 204; j++) {
        printf("%02X ", rs_out[j]);
    }
    printf("\n");

    /* Number of errors to insert */
    k = random() & 0x7F;

    for (j=0; j<k; ++j) {
      rs_out[random() % 204] = (random() & 0xFF);
    }

    rsdec_204(rs_in, rs_out);
    printf("Decoded data: ");
    for (j = 0; j < 188; j++) {
        printf("%02X ", rs_in[j]);
    }
    printf("\n");
  }
  return 0;
}
