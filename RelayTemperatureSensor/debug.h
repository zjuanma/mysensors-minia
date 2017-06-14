#ifndef DEBUG_H
#define DEBUG_H

#ifdef DEBUG
void debugPrint(const char *fmt, ... ) {
  char fmtBuffer[300];
  va_list args;
  va_start (args, fmt );
  va_end (args);
  vsnprintf_P(fmtBuffer, 299, fmt, args);
  va_end (args);
  Serial.print(fmtBuffer);
  Serial.flush();
}
#endif

#ifdef DEBUG
#define debug(x,...) debugPrint(x, ##__VA_ARGS__)
#else
#define debug(x,...)
#endif


#endif
