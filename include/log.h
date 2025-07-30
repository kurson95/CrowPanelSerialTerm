#ifndef LOG_WRAPPER_H
#define LOG_WRAPPER_H

#include <Arduino.h>
#include <stdarg.h>

#define ENABLE_DEBUG false

enum LogType
{
  LOG_INFO,
  LOG_ERROR,
  LOG_DEBUG,
  LOG_INMSG,
  LOG_OUTMSG,
  LOG_ACK,
  LOG_NACK,
  LOG_CMD
};

class Logger
{
public:
  Logger(Stream &out) : output(out), debugEnabled(ENABLE_DEBUG) {}

  void enableDebug(bool enable)
  {
    debugEnabled = enable;
  }

  void log(LogType type, const String &message)
  {
    if (type == LOG_DEBUG && !debugEnabled)
      return;
    output.print(getPrefix(type));
    output.println(message);
    resetPrefix();
  }

  void logf(LogType type, const char *fmt, ...)
  {
    if (type == LOG_DEBUG && !debugEnabled)
      return;

    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    output.print(getPrefix(type));
    output.println(buf);
    resetPrefix();
  }

  void logChar(LogType type, char c)
  {
    if (type == LOG_DEBUG && !debugEnabled)
      return;

    if (!prefixPrinted || type != currentType)
    {
      output.print(getPrefix(type));
      prefixPrinted = true;
      currentType = type;
    }

    output.print(c);

    if (c == '\n')
    {
      prefixPrinted = false;
    }
  }

private:
  Stream &output;
  bool debugEnabled = true;
  bool prefixPrinted = false;
  LogType currentType = LOG_INFO;

  const char *getPrefix(LogType type)
  {
    switch (type)
    {
    case LOG_INFO:
      return "[INF] ";
    case LOG_ERROR:
      return "[ERR] ";
    case LOG_DEBUG:
      return "[DBG] ";
    case LOG_INMSG:
      return "[IN]  ";
    case LOG_OUTMSG:
      return "[OUT] ";
    case LOG_ACK:
      return "[ACK]";
    case LOG_NACK:
      return "[NACK]";
    case LOG_CMD:
      return "[CMD]";
    default:
      return "";
    }
  }

  void resetPrefix()
  {
    prefixPrinted = false;
  }
};

#endif // LOG_WRAPPER_H
