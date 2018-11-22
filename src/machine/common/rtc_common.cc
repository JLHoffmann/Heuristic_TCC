// EPOS RTC Mediator Common Package Implementation

#include <rtc.h>

__BEGIN_SYS

RTC_Common::Date::Date(const Second & seconds, unsigned long epoch_days)
{
    static int days_per_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

    Second tmp = seconds;

    _s = tmp % 60;
    tmp /= 60;
    _m = tmp % 60;
    tmp /= 60;
    _h = tmp % 24;
    tmp /= 24;
    tmp += epoch_days;
    for(_Y = 1; tmp - 365 > 0; _Y++, tmp -= 365)
        if((!(_Y % 4) && (_Y % 100)) || !(_Y % 400))
            tmp--;
    days_per_month[1] = 28;
    if((!(_Y % 4) && (_Y % 100)) || !(_Y % 400))
        days_per_month[1] = 29;
    for(_M = 1; tmp - days_per_month[_M] > 0; _M++, tmp -= days_per_month[_M]);
    _D = tmp;
}

RTC_Common::Second RTC_Common::Date::to_offset(unsigned long epoch_days) const
{
    Date tmp(*this);
    unsigned long long y, m, d, t;
    //Year
    y = tmp._Y;
    //Month of year
    m = tmp._M;
    //Day of month
    d = tmp._D;
    //January and February are counted as months 13 and 14 of the previous year
    if(m <= 2) {
        m += 12;
        y -= 1;
    }
    //Convert years to days
    t = (365 * y) + (y / 4) - (y / 100) + (y / 400);
    //Convert months to days
    t += (30 * m) + (3 * (m + 1) / 5) + d;
    //Unix time starts on January 1st, 1970
    t -= 719561;
    //Convert days to seconds
    t *= 86400;
    //Add hours, minutes and seconds
    t += (3600 * tmp._h) + (60 * tmp._m) + tmp._s;

    //Return Unix time
    //return t;
    //se tiver em Joinville (onde nem o tempo é certo)
    return t;
}


__END_SYS
