import time
import SDL_DS3231 # See file for details. GitHub repo: https://github.com/switchdoclabs/RTC_SDL_DS3231


rtc = SDL_DS3231.SDL_DS3231(1, 0x68)
rtc.write_now()


t = time.perf_counter() + 1


while True:
    if time.perf_counter() > t:
        t = time.perf_counter() + 1
        
        print("Time now: {}".format(time.strftime("%Y:%m:%d %H:%M:%S")))
        print("RTC Time: {}\n".format(rtc.read_datetime()))
 
