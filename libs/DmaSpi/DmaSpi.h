#ifndef DMASPI_H
#define DMASPI_H

#include <teensy.h>
#include <util/atomic.h>

#if(!defined(__arm__) && defined(TEENSYDUINO))
  #error This library is for teensyduino 1.21 on Teensy 3.0, 3.1 and Teensy LC only.
#endif

#include <SPI.h>
#include "DMAChannel.h"

//#define DEBUG_DMASPI 1

#if defined(DEBUG_DMASPI)
  #include "debug.hpp"
  #define DMASPI_PRINT( x ) do { debug x ; } while (0);
  //#define DMASPI_PRINT(x) do {Serial.printf x ; Serial.flush();} while (0);
#else
  #define DMASPI_PRINT(x) do {} while (0);
#endif

namespace DmaSpiDetail
{
  template<uint8_t Channel>
  struct ChannelTraits;

#if defined(KINETISK)
/*
class DmaSpi0 : public AbstractDmaSpi<DmaSpi0>
{
public:
  static void begin_setup_txChannel_impl()
  {
	txChannel_.disable();
	txChannel_.destination((volatile uint8_t&)SPI0_PUSHR);
	txChannel_.disableOnCompletion();
	txChannel_.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_TX);
  }

  static void begin_setup_rxChannel_impl()
  {
	txChannel_.disable();
	rxChannel_.source((volatile uint8_t&)SPI0_POPR);
	rxChannel_.disableOnCompletion();
	rxChannel_.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_RX);
	rxChannel_.attachInterrupt(rxIsr_);
	rxChannel_.interruptAtCompletion();
  }

  static void pre_cs_impl()
  {
	SPI0_SR = 0xFF0F0000;
	SPI0_RSER = SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;
  }

  static void post_cs_impl()
  {
	rxChannel_.enable();
	txChannel_.enable();
  }

  static void post_finishCurrentTransfer_impl()
  {
	SPI0_RSER = 0;
	SPI0_SR = 0xFF0F0000;
  }

private:
};
*/
#elif defined(KINETISL)

  template<>
  struct ChannelTraits<0>
  {
  	template<typename DmaSpi>
    static void begin_setup_txChannel_impl(DmaSpi& dmaSpi)
    {
  	  dmaSpi.txChannel_.disable();
	  dmaSpi.txChannel_.destination((volatile uint8_t&)SPI0_DL);
	  dmaSpi.txChannel_.disableOnCompletion();
	  dmaSpi.txChannel_.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_TX);
    }

  	template<typename DmaSpi>
    static void begin_setup_rxChannel_impl(DmaSpi& dmaSpi)
    {
	  dmaSpi.txChannel_.disable();
	  dmaSpi.rxChannel_.source((volatile uint8_t&)SPI0_DL);
	  dmaSpi.rxChannel_.disableOnCompletion();
	  dmaSpi.rxChannel_.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_RX);
	  dmaSpi.rxChannel_.attachInterrupt(libcpp__mem_fn(&DmaSpi::rxIsr_, dmaSpi));
	  dmaSpi.rxChannel_.interruptAtCompletion();
    }

  	template<typename DmaSpi>
    static void pre_cs_impl(DmaSpi& dmaSpi)
    {
	  // disable SPI and enable SPI DMA requests
	  SPI0_C1 &= ~(SPI_C1_SPE);
	  SPI0_C2 |= SPI_C2_TXDMAE | SPI_C2_RXDMAE;
    }

  	template<typename DmaSpi>
    static void post_cs_impl(DmaSpi& dmaSpi)
    {
	  dmaSpi.rxChannel_.enable();
	  dmaSpi.txChannel_.enable();
    }

  	template<typename DmaSpi>
    static void post_finishCurrentTransfer_impl(DmaSpi& dmaSpi)
    {
	  SPI0_C2 = 0;
	  dmaSpi.txChannel_.clearComplete();
	  dmaSpi.rxChannel_.clearComplete();
    }
  };

  template<>
  struct ChannelTraits<1>
  {
  	template<typename DmaSpi>
    static void begin_setup_txChannel_impl(DmaSpi& dmaSpi)
    {
      dmaSpi.txChannel_.disable();
      dmaSpi.txChannel_.destination((volatile uint8_t&)SPI1_DL);
      dmaSpi.txChannel_.disableOnCompletion();
      dmaSpi.txChannel_.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI1_TX);
    }

  	template<typename DmaSpi>
    static void begin_setup_rxChannel_impl(DmaSpi& dmaSpi)
    {
      dmaSpi.txChannel_.disable();
      dmaSpi.rxChannel_.source((volatile uint8_t&)SPI1_DL);
      dmaSpi.rxChannel_.disableOnCompletion();
      dmaSpi.rxChannel_.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI1_RX);
      dmaSpi.rxChannel_.attachInterrupt(libcpp__mem_fn(&DmaSpi::rxIsr_, dmaSpi));
      dmaSpi.rxChannel_.interruptAtCompletion();
    }

  	template<typename DmaSpi>
    static void pre_cs_impl(DmaSpi& dmaSpi)
    {
      // disable SPI and enable SPI DMA requests
      SPI1_C1 &= ~(SPI_C1_SPE);
      SPI1_C2 |= SPI_C2_TXDMAE | SPI_C2_RXDMAE;
    }

  	template<typename DmaSpi>
    static void post_cs_impl(DmaSpi& dmaSpi)
    {
      dmaSpi.rxChannel_.enable();
      dmaSpi.txChannel_.enable();
    }

  	template<typename DmaSpi>
    static void post_finishCurrentTransfer_impl(DmaSpi& dmaSpi)
    {
      SPI1_C2 = 0;
      dmaSpi.txChannel_.clearComplete();
      dmaSpi.rxChannel_.clearComplete();
    }
  };

#endif // KINETISK else KINETISL

} // namespace DmaSpiDetail

/** \brief describes an SPI transfer
*
* Transfers are kept in a queue (intrusive linked list) until they are processed by the DmaSpi driver.
*
**/
class DmaSpiTransfer
{
public:
  /** \brief The Transfer's current state.
  *
  **/
  enum State
  {
	idle, /**< The Transfer is idle, the DmaSpi has not seen it yet. **/
	eDone, /**< The Transfer is done. **/
	pending, /**< Queued, but not handled yet. **/
	inProgress, /**< The DmaSpi driver is currently busy executing this Transfer. **/
	error /**< An error occured. **/
  };

  /** \brief Creates a Transfer object.
  * \param pSource pointer to the data source. If this is nullptr, the fill value is used instead.
  * \param transferCount the number of SPI transfers to perform.
  * \param pDest pointer to the data sink. If this is nullptr, data received from the slave will be discarded.
  * \param fill if pSource is nullptr, this value is sent to the slave instead.
  * \param cs pointer to a chip select object.
  *   If not nullptr, cs->select() is called when the Transfer is started and cs->deselect() is called when the Transfer is finished.
  **/
  DmaSpiTransfer(const uint8_t* pSource = nullptr,
			  const uint16_t& transferCount = 0,
			  volatile uint8_t* pDest = nullptr,
			  const uint8_t& fill = 0
  ) : m_state(State::idle),
	m_pSource(pSource),
	m_transferCount(transferCount),
	m_pDest(pDest),
	m_fill(fill)
  {
	  DMASPI_PRINT(("Transfer @ %p\n", this));
  };

  /** \brief Check if the Transfer is busy, i.e. may not be modified.
  **/
  bool busy() const {return ((m_state == State::pending) || (m_state == State::inProgress) || (m_state == State::error));}

  /** \brief Check if the Transfer is done.
  **/
  bool done() const {return (m_state == State::eDone);}

//      private:
  volatile State m_state;
  const uint8_t* m_pSource;
  uint16_t m_transferCount;
  volatile uint8_t* m_pDest;
  uint8_t m_fill;
};

template<typename Spi, typename Csel, uint32_t Clock = 4000000, uint8_t BitOrder = MSBFIRST, uint8_t DataMode = SPI_MODE0 >
class DmaSpi
{
  static uint8_t constexpr channel = Spi::channel;

  using ChannelTraits = DmaSpiDetail::ChannelTraits< channel >;
  friend ChannelTraits;

  public:
    using Transfer = DmaSpiTransfer;

    #if defined(KINETISK)
      typedef KINETISK_SPI_t SPI_t;
    #elif defined(KINETISL)
      typedef KINETISL_SPI_t SPI_t;
    #else
      #error I do not know how to handle your chip: neither KINETISK nor KINETISL defined.
    #endif

   /** \brief arduino-style initialization.
     *
     * During initialization, two DMA channels are allocated. If that fails, this function returns false.
     * If the channels could be allocated, those DMA channel fields that don't change during DMA SPI operation
     * are initialized to the values they will have at runtime.
     *
     * \return true if initialization was successful; false otherwise.
     * \see end()
    **/
    DmaSpi( Spi& spi )
		: spi_( spi )
    {
      DMASPI_PRINT(("DmaSpi::begin() : "));

      Csel::output();
      Csel::set(true);

      state_ = eStopped;
      // tx: known destination (SPI), no interrupt, finish silently
      begin_setup_txChannel();
      if (txChannel_.error())
      {
        DMASPI_PRINT(("tx channel error\n"));
		return;
      }

      // rx: known source (SPI), interrupt on completion
      begin_setup_rxChannel();
      if (rxChannel_.error())
      {
        DMASPI_PRINT(("rx channel error\n"));
        return;
      }
    }

    void begin_setup_txChannel() { ChannelTraits::begin_setup_txChannel_impl(*this); }
    void begin_setup_rxChannel() { ChannelTraits::begin_setup_rxChannel_impl(*this); }

    /** \brief Allow the DMA SPI to start handling Transfers. This must be called after begin().
     * \see running()
     * \see busy()
     * \see stop()
     * \see stopping()
     * \see stopped()
    **/
    void start()
    {
      DMASPI_PRINT(("DmaSpi::start() : state_ = "));
      switch(state_)
      {
        case eStopped:
          DMASPI_PRINT(("eStopped\n"));
          state_ = eRunning;
          beginPendingTransfer();
          break;

        case eRunning:
          DMASPI_PRINT(("eRunning\n"));
          break;

        case eStopping:
          DMASPI_PRINT(("eStopping\n"));
          state_ = eRunning;
          break;

        default:
          DMASPI_PRINT(("unknown\n"));
          state_ = eError;
          break;
      }
    }

    /** \brief check if the DMA SPI is in running state.
     * \return true if the DMA SPI is in running state, false otherwise.
     * \see start()
     * \see busy()
     * \see stop()
     * \see stopping()
     * \see stopped()
    **/
    bool running() {return state_ == eRunning;}

    /** \brief register a Transfer to be handled by the DMA SPI.
     * \return false if the Transfer had an invalid transfer count (zero or greater than 32767), true otherwise.
     * \post the Transfer state is Transfer::State::pending, or Transfer::State::error if the transfer count was invalid.
    **/
    bool registerTransfer(Transfer& transfer)
    {
      DMASPI_PRINT(("DmaSpi::registerTransfer(%p)\n", &transfer));
      if ((transfer.busy())
       || (transfer.m_transferCount == 0) // no zero length transfers allowed
       || (transfer.m_transferCount >= 0x8000)) // max CITER/BITER count with ELINK = 0 is 0x7FFF, so reject
      {
        DMASPI_PRINT(("  Transfer is busy or invalid, dropped\n"));
        transfer.m_state = Transfer::State::error;
        return false;
      }
      addTransferToQueue(transfer);
      if ((state_ == eRunning) && (!busy()))
      {
        DMASPI_PRINT(("  starting transfer\n"));
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
          beginPendingTransfer();
        }
      }
      return true;
    }

    /** \brief Check if the DMA SPI is busy, which means that it is currently handling a Transfer.
     \return true if a Transfer is being handled.
     * \see start()
     * \see running()
     * \see stop()
     * \see stopping()
     * \see stopped()
    **/
    bool busy()
    {
      return (m_pTransfer != nullptr) && (m_pTransfer->m_state != Transfer::State::pending);
    }

    /** \brief Request the DMA SPI to stop handling Transfers.
     *
     * The stopping driver may finish a current Transfer, but it will then not start a new, pending one.
     * \see start()
     * \see running()
     * \see busy()
     * \see stopping()
     * \see stopped()
    **/
    void stop()
    {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        switch(state_)
        {
          case eStopped:
            break;
          case eRunning:
            if (busy())
            {
              state_ = eStopping;
            }
            else
            {
              // this means that the DMA SPI simply has nothing to do
              state_ = eStopped;
            }
            break;
          case eStopping:
            break;
          default:
            state_ = eError;
            break;
        }
      }
    }

    /** \brief See if the DMA SPI is currently switching from running to stopped state
     * \return true if the DMA SPI is switching from running to stopped state
     * \see start()
     * \see running()
     * \see busy()
     * \see stop()
     * \see stopped()
    **/
    bool stopping() { return (state_ == eStopping); }

    /** \brief See if the DMA SPI is stopped
    * \return true if the DMA SPI is in stopped state, i.e. not handling pending Transfers
     * \see start()
     * \see running()
     * \see busy()
     * \see stop()
     * \see stopping()
    **/
    bool stopped() { return (state_ == eStopped); }

    /** \brief Shut down the DMA SPI
     *
     * Deallocates DMA channels and sets the internal state to error (this might not be an intelligent name for that)
     * \see begin()
    **/
    ~DmaSpi()
    {
	  state_ = eError;
    }

    /** \brief get the last value that was read from a slave, but discarded because the Transfer didn't specify a sink
    **/
    uint8_t devNull()
    {
      return m_devNull;
    }

  protected:
    enum EState
    {
      eStopped,
      eRunning,
      eStopping,
      eError
    };

    void addTransferToQueue(Transfer& transfer)
    {
      transfer.m_state = Transfer::State::pending;
      DMASPI_PRINT(("  DmaSpi::addTransferToQueue() : queueing transfer\n"));
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
      	m_pTransfer = &transfer;
      }
    }

    void post_finishCurrentTransfer() { ChannelTraits::post_finishCurrentTransfer_impl(*this); }

    void finishCurrentTransfer()
    {
	  Csel::set(true);
      spi_.endTransaction();
      m_pTransfer->m_state = Transfer::State::eDone;
      DMASPI_PRINT(("  finishCurrentTransfer() @ %p\n", m_pTransfer));
      m_pTransfer = nullptr;
      post_finishCurrentTransfer();
    }

    void rxIsr_()
    {
      DMASPI_PRINT(("DmaSpi::rxIsr_()\n"));
      rxChannel_.clearInterrupt();
      // end current transfer: deselect and mark as done
      finishCurrentTransfer();

      DMASPI_PRINT(("  state = "));
      switch(state_)
      {
        case eStopped: // this should not happen!
        DMASPI_PRINT(("eStopped\n"));
          state_ = eError;
          break;
        case eRunning:
          DMASPI_PRINT(("eRunning\n"));
          beginPendingTransfer();
          break;
        case eStopping:
          DMASPI_PRINT(("eStopping\n"));
          state_ = eStopped;
          break;
        case eError:
          DMASPI_PRINT(("eError\n"));
          break;
        default:
          DMASPI_PRINT(("eUnknown\n"));
          state_ = eError;
          break;
      }
    }

    void pre_cs() { ChannelTraits::pre_cs_impl(*this); }
    void post_cs() { ChannelTraits::post_cs_impl(*this); }

    void beginPendingTransfer()
    {
      if (m_pTransfer == nullptr)
      {
        DMASPI_PRINT(("DmaSpi::beginNextTransfer: no pending transfer\n"));
        return;
      }

      DMASPI_PRINT(("DmaSpi::beginNextTransfer: starting transfer @ %p\n", m_pTransfer));
      m_pTransfer->m_state = Transfer::State::inProgress;

      // configure Rx DMA
      if (m_pTransfer->m_pDest != nullptr)
      {
        // real data sink
        DMASPI_PRINT(("  real sink\n"));
        rxChannel_.destinationBuffer(m_pTransfer->m_pDest,
                                        m_pTransfer->m_transferCount);
      }
      else
      {
        // dummy data sink
        DMASPI_PRINT(("  dummy sink\n"));
        rxChannel_.destination(m_devNull);
        rxChannel_.transferCount(m_pTransfer->m_transferCount);
      }

      // configure Tx DMA
      if (m_pTransfer->m_pSource != nullptr)
      {
        // real data source
        DMASPI_PRINT(("  real source\n"));
        txChannel_.sourceBuffer(m_pTransfer->m_pSource,
                                   m_pTransfer->m_transferCount);
      }
      else
      {
        // dummy data source
        DMASPI_PRINT(("  dummy source\n"));
        txChannel_.source(m_pTransfer->m_fill);
        txChannel_.transferCount(m_pTransfer->m_transferCount);
      }

      pre_cs();

      // Select Chip
      spi_.template beginTransaction<Clock, BitOrder, DataMode>();
      Csel::set(false);

      post_cs();
    }

    Spi& spi_;
    volatile EState state_ {};
    Transfer* volatile m_pTransfer {};
    volatile uint8_t m_devNull {};
    DMAChannel txChannel_;
    DMAChannel rxChannel_;
};

#endif // DMASPI_H
