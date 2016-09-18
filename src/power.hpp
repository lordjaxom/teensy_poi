#ifndef TEENSY_POI_POWER_HPP
#define TEENSY_POI_POWER_HPP

namespace tp {

    template< typename Pin, bool Invert = true >
    struct PowerGuard
    {
        PowerGuard()
        {
            Pin::output();
            disable();
        }

        PowerGuard( PowerGuard const& ) = delete;

        void enable()
        {
            Pin::set( !Invert );
        }

        void disable()
        {
            Pin::set( Invert );
        }
    };

} // namespace tp

#endif // TEENSY_POI_POWER_HPP
