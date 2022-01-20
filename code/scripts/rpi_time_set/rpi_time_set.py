import time
import SDL_DS3231 # See file for details. GitHub repo: https://github.com/switchdoclabs/RTC_SDL_DS3231


rtc = SDL_DS3231.SDL_DS3231(1, 0x68)

old = rtc.read_datetime()
rtc.write_now()
now = rtc.read_datetime()
print("RTC time was: {}".format(old))
print("RTC time now: {}\n\n".format(now))


t = time.perf_counter() + 1

"""
while True:
    if time.perf_counter() > t:
        t = time.perf_counter() + 1

        print("System time: {}".format(time.strftime("%Y:%m:%d %H:%M:%S")))
        print("RTC Time: {}\n".format(rtc.read_datetime()))
""" 
