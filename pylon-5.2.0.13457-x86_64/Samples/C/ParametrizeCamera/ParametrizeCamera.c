/*
  This sample illustrates how to read and write the different camera
  parameter types.
*/

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#ifdef __GNUC__
#   include <alloca.h>
#endif

#include <pylonc/PylonC.h>

#define CHECK( errc ) if ( GENAPI_E_OK != errc ) printErrorAndExit( errc )

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void);

void printErrorAndExit( GENAPIC_RESULT errc );
void demonstrateAccessibilityCheck( PYLON_DEVICE_HANDLE );
void demonstrateIntFeature( PYLON_DEVICE_HANDLE );
void demonstrateInt32Feature( PYLON_DEVICE_HANDLE );
void demonstrateFloatFeature( PYLON_DEVICE_HANDLE );
void demonstrateBooleanFeature( PYLON_DEVICE_HANDLE );
void demonstrateFromStringToString( PYLON_DEVICE_HANDLE );
void demonstrateEnumFeature( PYLON_DEVICE_HANDLE );
void demonstrateCommandFeature( PYLON_DEVICE_HANDLE );


int main(void)
{
    GENAPIC_RESULT              res;           /* Return value of pylon methods. */
    size_t                      numDevices;    /* Number of available devices. */
    PYLON_DEVICE_HANDLE         hDev;          /* Handle for the pylon device. */

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all camera devices. You must call
    PylonEnumerateDevices() before creating a device. */
    res = PylonEnumerateDevices( &numDevices );
    CHECK(res);
    if ( 0 == numDevices )
    {
        fprintf( stderr, "No devices found.\n" );
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Get a handle for the first device found.  */
    res = PylonCreateDeviceByIndex( 0, &hDev );
    CHECK(res);

    /* Before using the device, it must be opened. Open it for configuring
    parameters and for grabbing images. */
    res = PylonDeviceOpen( hDev, PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM );
    CHECK(res);

    /* Print out the name of the camera we are using. */
    {
        char buf[256];
        size_t siz = sizeof(buf);
        _Bool isReadable;

        isReadable = PylonDeviceFeatureIsReadable(hDev, "DeviceModelName");
        if ( isReadable )
        {
            res = PylonDeviceFeatureToString( hDev, "DeviceModelName", buf, &siz );
            CHECK(res);
            printf("Using camera %s\n", buf);
        }
    }

    /* Demonstrate how to check the accessibility of a feature. */
    demonstrateAccessibilityCheck( hDev );
    puts("");

    /* Demonstrate how to handle integer camera parameters. */
    demonstrateIntFeature( hDev );
    puts("");
    demonstrateInt32Feature( hDev );
    puts("");

    /* Demonstrate how to handle floating point camera parameters. */
    demonstrateFloatFeature( hDev );
    puts("");

    /* Demonstrate how to handle boolean camera parameters. */
    demonstrateBooleanFeature( hDev );
    puts("");

    /* Each feature can be read as a string and also set as a string. */
    demonstrateFromStringToString( hDev );
    puts("");

    /* Demonstrate how to handle enumeration camera parameters. */
    demonstrateEnumFeature( hDev );
    puts("");

    /* Demonstrate how to execute actions. */
    demonstrateCommandFeature( hDev );



    /* Clean up. Close and release the pylon device. */

    res = PylonDeviceClose( hDev );
    CHECK(res);
    res = PylonDestroyDevice ( hDev );
    CHECK(res);


    /* Shut down the pylon runtime system. Don't call any pylon method after
       calling PylonTerminate(). */
    PylonTerminate();
    pressEnterToExit();

    return EXIT_SUCCESS;
}

/* This function demonstrates how to retrieve the error message for the last failed
   function call. */
void printErrorAndExit( GENAPIC_RESULT errc )
{
    char *errMsg;
    size_t length;

    /* Retrieve the error message.
    ... First find out how big the buffer must be, */
    GenApiGetLastErrorMessage( NULL, &length );
    errMsg = (char*) malloc( length );
    /* ... and retrieve the message. */
    GenApiGetLastErrorMessage( errMsg, &length );

    fprintf( stderr, "%s (%#08x).\n", errMsg, (unsigned int) errc);
    free( errMsg);

    /* Retrieve the more details about the error
    ... First find out how big the buffer must be, */
    GenApiGetLastErrorDetail( NULL, &length );
    errMsg = (char*) malloc( length );
    /* ... and retrieve the message. */
    GenApiGetLastErrorDetail( errMsg, &length );

    fprintf( stderr, "%s\n", errMsg);
    free( errMsg);

    PylonTerminate();  /* Releases all pylon resources */
    pressEnterToExit();

    exit(EXIT_FAILURE);
}

/* This function demonstrates how to check the presence, readability, and writability
   of a feature. */
void demonstrateAccessibilityCheck( PYLON_DEVICE_HANDLE hDev )
{
    _Bool val;  /* Output of the check functions */

    /* Check to see if a feature is implemented at all. */
    val = PylonDeviceFeatureIsImplemented(hDev, "Width");
    printf("The 'Width' feature %s implemented\n", val ? "is" : "isn't");
    val = PylonDeviceFeatureIsImplemented(hDev, "MyCustomFeature");
    printf("The 'MyCustomFeature' feature %s implemented\n", val ? "is" : "isn't");


    /* Although a feature is implemented by the device, it might not be available
       with the device in its current state. Check to see if the feature is currently
       available. The PylonDeviceFeatureIsAvailable sets val to 0 if either the feature
       is not implemented or if the feature is not currently available. */

    val = PylonDeviceFeatureIsAvailable(hDev, "BinningVertical");
    printf("The 'BinningVertical' feature %s available\n", val ? "is" : "isn't");

    /* If a feature is available, it could be read-only, write-only, or both
       readable and writable. Use the PylonDeviceFeatureIsReadable() and the
       PylonDeviceFeatureIsWritable() functions(). It is safe to call these functions
       for features that are currently not available or not implemented by the device.
       A feature that is not available or not implemented is neither readable nor writable.
       The readability and writability of a feature can change depending on the current
       state of the device. For example, the Width parameter might not be writable when
       the camera is acquiring images. */


    val = PylonDeviceFeatureIsReadable(hDev, "Width");
    printf("The 'Width' feature %s readable\n", val ? "is" : "isn't");
    val = PylonDeviceFeatureIsImplemented( hDev, "MyCustomFeature");
    printf("The 'MyCustomFeature' feature %s readable\n", val ? "is" : "isn't");

    val = PylonDeviceFeatureIsWritable( hDev, "Width");
    printf("The 'Width' feature %s writable\n", val ? "is" : "isn't");

    printf("\n");
}


/* This function demonstrates how to handle integer camera parameters. */
void demonstrateIntFeature( PYLON_DEVICE_HANDLE hDev )
{
    static const char   featureName[] = "Width";  /* Name of the feature used in this sample: AOI Width */
    int64_t             val, min, max, incr;      /* Properties of the feature */
    GENAPIC_RESULT      res;                      /* Return value */


    if ( PylonDeviceFeatureIsReadable(hDev, featureName) )
    {
        /*
          Query the current value, the allowed value range, and the increment of the feature.
          For some integer features, you are not allowed to set every value within the
          value range. For example, for some cameras the Width parameter must be a multiple
          of 2. These constraints are expressed by the increment value. Valid values
          follow the rule: val >= min && val <= max && val == min + n * inc. */
        res = PylonDeviceGetIntegerFeatureMin( hDev, featureName, &min );  /* Get the minimum value. */
        CHECK(res);
        res = PylonDeviceGetIntegerFeatureMax( hDev, featureName, &max );  /* Get the maximum value. */
        CHECK(res);
        res = PylonDeviceGetIntegerFeatureInc( hDev, featureName, &incr);  /* Get the increment value. */
        CHECK(res);
        res = PylonDeviceGetIntegerFeature( hDev, featureName, &val );     /* Get the current value. */
        CHECK(res);

#if __STDC_VERSION__ >= 199901L || defined(__GNUC__)
        printf("%s: min= %lld  max= %lld  incr=%lld  Value=%lld\n", featureName, (long long ) min, (long long ) max, (long long ) incr, (long long ) val );
#else
        printf("%s: min= %I64d  max= %I64d  incr=%I64d  Value=%I64d\n", featureName, min, max, incr, val );
#endif

        if ( PylonDeviceFeatureIsWritable(hDev, featureName) )
        {
            /* Set the Width half-way between minimum and maximum. */
            res = PylonDeviceSetIntegerFeature( hDev, featureName, min + (max - min) / incr / 2 * incr );
            CHECK(res);
        }
        else
            fprintf(stderr, "The %s feature is not writable.\n", featureName );
    }
    else
        fprintf(stderr, "The %s feature is not readable.\n", featureName );
}


/* The integer functions illustrated above take 64 bit integers as output parameters. There are variants
   of the integer functions that accept 32 bit integers instead. The Get.... functions return
   an error when the value returned by the device doesn't fit into a 32 bit integer. */
void demonstrateInt32Feature( PYLON_DEVICE_HANDLE hDev )
{
    static const char   featureName[] = "Height";  /* Name of the feature used in this sample: AOI height */
    int32_t             val, min, max, incr;       /* Properties of the feature */
    GENAPIC_RESULT      res;                       /* Return value */


    if ( PylonDeviceFeatureIsReadable(hDev, featureName) )
    {
        /*
           Query the current value, the allowed value range, and the increment of the feature.
           For some integer features, you are not allowed to set every value within the
           value range. For example, for some cameras the Width parameter must be a multiple
           of 2. These constraints are expressed by the increment value. Valid values
           follow the rule: val >= min && val <= max && val == min + n * inc. */
        res = PylonDeviceGetIntegerFeatureMinInt32( hDev, featureName, &min );  /* Get the minimum value. */
        CHECK(res);
        res = PylonDeviceGetIntegerFeatureMaxInt32( hDev, featureName, &max );  /* Get the maximum value. */
        CHECK(res);
        res = PylonDeviceGetIntegerFeatureIncInt32( hDev, featureName, &incr);  /* Get the increment value. */
        CHECK(res);
        res = PylonDeviceGetIntegerFeatureInt32( hDev, featureName, &val );     /* Get the current value. */
        CHECK(res);
        printf("%s: min= %d  max= %d  incr=%d  Value=%d\n", featureName, min, max, incr, val );

        if ( PylonDeviceFeatureIsWritable(hDev, featureName) )
        {
            /* Set the value to half its maximum  */
            res = PylonDeviceSetIntegerFeatureInt32( hDev, featureName, min + (max - min) / incr / 2 * incr );
            CHECK(res);
        }
        else
            fprintf(stderr, "The %s feature is not writable.\n", featureName );
    }
    else
        fprintf(stderr, "The %s feature is not readable.\n", featureName );
}


/* Some features are floating point features. This function illustrates how to set and get floating
   point parameters. */
void demonstrateFloatFeature( PYLON_DEVICE_HANDLE hDev )
{
    static const char   featureName[] = "Gamma";  /* The name of the feature used */
    _Bool                isWritable;               /* Is the feature writable? */
    double              min, max, value;          /* Value range and current value */
    GENAPIC_RESULT      res;                      /* Return value */

    if ( PylonDeviceFeatureIsReadable(hDev, featureName) )
    {
        /* Query the value range and the current value. */
        res = PylonDeviceGetFloatFeatureMin( hDev, featureName, &min);
        CHECK(res);
        res = PylonDeviceGetFloatFeatureMax( hDev, featureName, &max);
        CHECK(res);
        res = PylonDeviceGetFloatFeature( hDev, featureName, &value );
        CHECK(res);

        printf("%s: min = %4.2f, max = %4.2f, value = %4.2f\n", featureName, min, max, value );

        /* Set the value to half its maximum. */
        isWritable = PylonDeviceFeatureIsWritable(hDev, featureName);
        if ( isWritable )
        {
            value = 0.5 * ( min + max );
            printf("Setting %s to %4.2f\n", featureName, value );
            res = PylonDeviceSetFloatFeature( hDev, featureName, value );
            CHECK(res);
        }
        else
            fprintf(stderr, "The %s feature is not writable.\n", featureName );
    }
    else
        fprintf(stderr, "The %s feature is not readable.\n", featureName );
}


/* Some features are boolean features that can be switched on and off.
   This function illustrates how to access boolean features. */
void demonstrateBooleanFeature( PYLON_DEVICE_HANDLE hDev )
{
    static const char   featureName[] = "GammaEnable"; /* The name of the feature */
    _Bool                isWritable;                    /* Is the feature writable? */
    _Bool                value;                         /* The value of the feature */
    GENAPIC_RESULT      res;                           /* Return value */

    /* Check to see if the feature is writable. */
    isWritable = PylonDeviceFeatureIsWritable(hDev, featureName);

    if ( isWritable )
    {
        /* Retrieve the current state of the feature. */
        res = PylonDeviceGetBooleanFeature( hDev, featureName, &value);
        CHECK(res);
        printf("The %s features is %s\n", featureName, value ? "on" : "off" );

        /* Set a new value. */
        value = (_Bool) !value;  /* New value */
        printf("Switching the %s feature %s\n", featureName, value ? "on" : "off" );
        res = PylonDeviceSetBooleanFeature( hDev, featureName, value );
        CHECK(res);

    }
    else
        printf("The %s feature isn't writable\n", featureName );
}


/*
  Regardless of the parameter's type, any parameter value can be retrieved as a string. Each parameter
  can be set by passing in a string correspondingly. This function illustrates how to set and get the
  Width parameter as string. As demonstrated above, the Width parameter is of the integer type.
  */
void demonstrateFromStringToString( PYLON_DEVICE_HANDLE hDev )
{
    static const char   featureName[] = "Width";   /* The name of the feature */

    size_t              len;
    char*               buf;
    char                smallBuf[1];
    char                properBuf[32];
    GENAPIC_RESULT      res;                       /* Return value */

    /* Get the value of a feature as a string. Normally getting the value consits of 3 steps:
       1.) Determine the required buffer size.
       2.) Allocate the buffer.
       3.) Retrieve the value. */
    /* ... Get the required buffer size. The size is queried by
           passing a NULL pointer as a pointer to the buffer. */
    res = PylonDeviceFeatureToString( hDev, featureName, NULL, &len );
    CHECK(res);
    /* ... Len is set to the required buffer size (terminating zero included).
           Allocate the memory and retrieve the string. */
    buf = (char*) alloca( len );
    res = PylonDeviceFeatureToString( hDev, featureName, buf, &len );
    CHECK( res );

    printf("%s: %s\n", featureName, buf );

    /* You are not necessarily required to query the buffer size in advance. If the buffer is
       big enough, passing in a buffer and a pointer to its length will work.
       When the buffer is too small, an error is returned. */

    /* Passing in a buffer that is too small */
    len = sizeof (smallBuf);
    res = PylonDeviceFeatureToString( hDev, featureName, smallBuf, &len );
    if ( res == GENAPI_E_INSUFFICIENT_BUFFER )
    {
        /* The buffer was too small. The required size is indicated by len. */
        printf("Buffer is too small for the value of '%s'. The required buffer size is %d\n", featureName, (int) len );
    }
    else
        CHECK(res);  /* Unexpected return value */

    /* Passing in a buffer with sufficient size. */
    len = sizeof (properBuf );
    res = PylonDeviceFeatureToString( hDev, featureName, properBuf, &len );
    CHECK(res);


    /* A feature can be set as a string using the PylonDeviceFeatureFromString() function.
       If the content of a string can not be converted to the type of the feature, an
       error is returned. */
    res = PylonDeviceFeatureFromString( hDev, featureName, "fourty-two"); /* Can not be converted to an integer */
    if ( res != GENAPI_E_OK )
    {
        /* Print out an error message. */
        size_t l;
        char *msg;
        GenApiGetLastErrorMessage( NULL, &l ); /* Retrieve buffer size for the error message */
        msg = (char *)malloc(l);             /* Provide memory */
        GenApiGetLastErrorMessage( msg, &l );  /* Retrieve the message */
        printf("%s\n", msg );
        free(msg);
    }
}


/* There are camera features that behave like enumerations. These features can take a value from a fixed
   set of possible values. One example is the pixel format feature. This function illustrates how to deal with
   enumeration features.

*/
void demonstrateEnumFeature( PYLON_DEVICE_HANDLE hDev )
{
    char                value[64];                     /* The current value of the feature */
    size_t              len;                           /* The length of the string */
    GENAPIC_RESULT      res;                           /* Return value */
    _Bool                isWritable,
                        supportsMono8,
                        supportsYUV422Packed,
                        supportsMono16;


    /* The allowed values for an enumeration feature are represented as strings. Use the
    PylonDeviceFeatureFromString() and PylonDeviceFeatureToString() methods for setting and getting
    the value of an enumeration feature. */


    /* Get the current value of the enumeration feature. */
    len = sizeof(value);
    res = PylonDeviceFeatureToString( hDev, "PixelFormat", value, &len );
    CHECK(res);

    printf("PixelFormat: %s\n", value);

    /*
      For an enumeration feature, the pylon Viewer's "Feature Documentation" window lists the the
      names of the possible values. Some of the values might not be supported by the device.
      To check if a certain "SomeValue" value for a "SomeFeature" feature can be set, call the
      PylonDeviceFeatureIsAvailable() function with "EnumEntry_SomeFeature_SomeValue" as an argument.
    */
    /* Check to see if the Mono8 pixel format can be set. */
    supportsMono8 = PylonDeviceFeatureIsAvailable(hDev, "EnumEntry_PixelFormat_Mono8");
    printf("Mono8 %s a supported value for the PixelFormat feature\n", supportsMono8 ? "is" : "isn't");

    /* Check to see if the YUV422Packed pixel format can be set. */
    supportsYUV422Packed = PylonDeviceFeatureIsAvailable(hDev, "EnumEntry_PixelFormat_YUV422Packed");
    printf("YUV422Packed %s a supported value for the PixelFormat feature\n", supportsYUV422Packed ? "is" : "isn't");

    /* Check to see if the Mono16 pixel format can be set. */
    supportsMono16 = PylonDeviceFeatureIsAvailable(hDev, "EnumEntry_PixelFormat_Mono16");
    printf("Mono16 %s a supported value for the PixelFormat feature\n", supportsMono16 ? "is" : "isn't");


    /* Before writing a value, we recommend checking to see if the enumeration feature is
       currently writable. */
    isWritable = PylonDeviceFeatureIsWritable(hDev, "PixelFormat");
    if ( isWritable )
    {
        /* The PixelFormat feature is writable, set it to one of the supported values. */
        if ( supportsMono16 )
        {
            printf("Setting PixelFormat to Mono16\n");
            res = PylonDeviceFeatureFromString( hDev, "PixelFormat", "Mono16" );
            CHECK(res);
        }
        else if ( supportsYUV422Packed )
        {
            printf("Setting PixelFormat to YUV422Packed\n");
            res = PylonDeviceFeatureFromString( hDev, "PixelFormat", "YUV422Packed" );
            CHECK(res);
        }
        else if ( supportsMono8 )
        {
            printf("Setting PixelFormat to Mono8\n");
            res = PylonDeviceFeatureFromString( hDev, "PixelFormat", "Mono8" );
            CHECK(res);
        }

        /* Reset the PixelFormat feature to its previous value. */
        PylonDeviceFeatureFromString( hDev, "PixelFormat", value );
    }

}


/* There are camera features, such as starting image acquisition, that represent a command.
   This function that loads the factory settings, illustrates how to execute a command feature.  */
void demonstrateCommandFeature( PYLON_DEVICE_HANDLE hDev )
{
    GENAPIC_RESULT      res;  /* Return value. */

    /* Before executing the user set load command, the user set selector must be
       set to the default set. Since we are focusing on the command feature,
       we skip the recommended steps for checking the availability of the user set
       related features and values. */

    /* Choose the default configuration set (with one of the factory setups chosen). */
    res = PylonDeviceFeatureFromString( hDev, "UserSetSelector", "Default" );
    CHECK(res);

    /* Execute the user set load command. */
    printf("Loading the default set.\n");
    res = PylonDeviceExecuteCommandFeature( hDev, "UserSetLoad" );
    CHECK(res);
}

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void)
{
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

