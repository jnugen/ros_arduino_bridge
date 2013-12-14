//
// Configuration parameters
//

#define HAVE_MOTORS      // Enable the base motor code
//#undef HAVE_MOTORS     // Disable the base motor code

/* Define the motor controller library you are using */
#ifdef HAVE_MOTORS
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The HRBC ClubBot motor driver */
   #define HBRC_CLUBBOT

   /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
#endif

//#define HAVE_ENCODERS      // Enable the base encoder code
#undef HAVE_ENCODERS     // Disable the base encoder code

/* Define the encoder library you are using */
#ifdef HAVE_ENCODERS
   /* The RoboGaia encoder shield */
   //#define ROBOGAIA
#endif

//#define USE_PID  // Enable use of PID controller code
#undef USE_PID     // Disable use of PID controller code

#ifdef USE_PID
  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Define the attachment of any servos here */
#ifdef USE_SERVOS
   /* Number of servos */
   #define N_SERVOS 2

   /* List of servo pins */
   #define SERVO_PINS {3, 5}
#endif

//#define USE_SONAR  // Enable use of sonar
#undef USE_SONAR     // Disable use of sonar

// Serial port baud rate
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

