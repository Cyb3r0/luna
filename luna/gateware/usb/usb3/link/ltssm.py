#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause
""" Link Training and Status State Machine (LTSSM) gateware. """

#
# WARNING: This implementation is currently the minimum set of things that make
# the SerDes PHY work in some cases. This is not a complete design.
#

import math

from nmigen import *
from nmigen.lib.coding import Encoder


class LTSSMController(Elaboratable):
    """ Link Training and Status State Machine

    This state machine orchestrates USB bringup, link training, and power saving.
    It is implemented according to chaper 7.5 of the USB 3.2 specification [USB3.2r1; 7.5].

    Attributes
    ----------
    link_ready: Signal(), output
        Asserted when the link is ready to perform normal USB operations starting
        at the link layer. When de-asserted, data should not flow above the link layer.
    in_usb_reset: Signal(), input
        Should be asserted whenever a USB reset is detected; including a power-on reset (e.g.
        the PHY has yet to start up), or LFPS warm reset signaling.

    train_alignment: Signal(), output
        Asserted when the link is most ready to perform initial alignment training; e.g. while
        TSEQ test sequences are being exchanged. For some physical layer interfaces, this additional
        metadata helps with initial comma alignment.
    engage_terminations: Signal(), output
        When asserted, the physical layer should be applying link terminations.
    invert_rx_polarity: Signal(). output
        When asserted, indicates that the physical layer should invert all received data at the I/O
        boundary to counter a swapped Rx differential pair.

    perform_rx_detection: Signal(), output
        Asserted to request that the physical layer perform receiver detection, in which the link

    enable_scrambling: Signal(), output
        Asserted when the physical layer should be performing scrambling.


    """

    def __init__(self, ss_clock_frequency):
        self._clock_frequency = ss_clock_frequency

        #
        # I/O port.
        #
        self.link_ready            = Signal()
        self.in_usb_reset          = Signal()

        # SerDes / link control signals.
        self.train_alignment       = Signal()
        self.train_equalizer       = Signal()
        self.engage_terminations   = Signal()
        self.invert_rx_polarity    = Signal()

        # Receiver detection.
        self.perform_rx_detection  = Signal()
        self.link_partner_detected = Signal()

        # LFPS detection / emission.
        self.lfps_polling_detected = Signal()
        self.send_lfps_polling     = Signal()

        # Training set detection signals.
        self.tseq_detected         = Signal()
        self.ts1_detected          = Signal()
        self.inverted_ts1_detected = Signal()
        self.ts2_detected          = Signal()

        # Training set generation signals.
        self.send_tseq_burst       = Signal()
        self.send_ts1_burst        = Signal()
        self.send_ts2_burst        = Signal()
        self.ts_burst_complete     = Signal()

        # Late-stage link management; physical layer control.
        self.enable_scrambling     = Signal()
        self.logical_idle_detected = Signal()



    def elaborate(self, platform):
        m = Module()

        #
        #  Control Signals
        #
        m.d.comb += [
            # For now, continuously train word alignment. If this works, this signal will go away.
            self.train_alignment      .eq(1),
            self.train_equalizer      .eq(1),

            # Engage our receive terminations, unless the current state directs otherwise.
            self.engage_terminations  .eq(1)
        ]


        #
        # Timeout tracking.
        #

        # Create a timer that can count up to at least 360mS, the largest LTSSM state timeout.
        # [USB 3.2r1: 7.5]
        cycles_in_360mS = int(math.ceil(360e-3 * self._clock_frequency))
        cycles_in_state = Signal(range(cycles_in_360mS + 1))

        # Count by default; this will be automatically cleared on state transitions.
        m.d.ss += cycles_in_state.eq(cycles_in_state + 1)


        #
        # Asynchronous Training Sequence & LFPS Detectors
        #
        polling_seen  = Signal()
        ts2_seen      = Signal()

        with m.If(self.ts2_detected):
            m.d.ss += ts2_seen.eq(1)

        with m.If(self.lfps_polling_detected):
            m.d.ss += polling_seen.eq(1)


        #
        # FSM helpers.
        #

        # Create a list of tasks to perform on entry to a given state.
        # These will be automatically handled by transition_to_state()
        tasks_on_entry = {}


        def transition_to_state(state):
            """ FSM helper that handles transitions to the given state.

            Automatically handles any "on entry" conditions for the given state.
            """

            # Clear our "time-in-state" counter.
            m.d.ss += cycles_in_state.eq(0)

            # If we have any additional entry conditions for the given state, apply them.
            if state in tasks_on_entry:
                m.d.ss += tasks_on_entry[state]

            m.next = state


        def transition_on_timeout(timeout, *, to):
            """ FSM helper that adds a state transition that is automatically invoked after a timeout. """

            # Figure out how many cycles need to pass before we consider ourselves timed out.
            timeout_in_cycles = int(math.ceil(timeout * self._clock_frequency))

            # If we've reached that many cycles, transition to the target state.
            with m.If(cycles_in_state == timeout_in_cycles):
                transition_to_state(to)


        def handle_warm_resets():
            """ FSM helper that automatically moves back to the Rx.Detect.Reset state when appropriate."""

            # If we're in USB reset, we're actively receiving warm reset signaling; and we should reset
            # to the Rx.Detect.Reset state.
            with m.If(self.in_usb_reset):
                transition_to_state("Rx.Detect.Reset")


        #
        # Main Link Training and Status State Machine
        #
        with m.FSM(domain="ss"):

            # Rx.Detect.Reset -- we've just started link bringup post-reset; and are ready to
            # perform any necessary link configuration.
            with m.State("Rx.Detect.Reset"):

                # We'll wait in this state until our board is brought up, and we're not detecting
                # any Warm Reset LFPS signaling.
                with m.If(~self.in_usb_reset):
                    transition_to_state("Rx.Detect.Active")


            # Rx.Detect.Active -- we're now post-reset; and we're going to attempt to detect a
            # link partner, by checking for far-end receiver terminations. Basically, we try to
            # detect whether we're connected to another SuperSpeed transciever via a cable, so
            # we don't waste time performing link training if our link isn't there.
            with m.State("Rx.Detect.Active"):

                # For now, we'll always assume that we have a link partner present.
                # TODO: correctly detect a link partner and move to Polling.LFPS or Rx.Detect.Quiet.
                transition_to_state("Polling.LFPS")


            # Polling.LFPS -- now that we know there's someone listening on the other side, we'll
            # begin exchanging LFPS messages; giving the two sides the opportunity to sync up and
            # establish initial DC characteristics. [USB 3.2r1: 7.5.4.3]
            with m.State("Polling.LFPS"):

                # Continuously send our LFPS polling.
                m.d.comb += self.send_lfps_polling.eq(1)

                # Once we see a valid LFPS polling response, we can move to exchanging test sets.
                with m.If(self.lfps_polling_detected):
                    transition_to_state("Polling.RxEQ")

                # If we've never seen polling, we'll exit to Compliance once this passes. [USB 3.2r1: 7.5.4.3]
                with m.If(~polling_seen):
                    transition_on_timeout(360e-3, to="Compliance")


            # Polling.RxEQ -- we've now seen the other side of our link, and are ready to initialize
            # communications. We'll bring our link online, start sending our first Training Set (TSEQ),
            # and give the PHY time to achieve DC equalization.
            # [USB 3.2.r1: 7.5.4.7]
            with m.State("Polling.RxEQ"):
                handle_warm_resets()

                # Ensure we enter our primary training sequence with a fresh view of whether we've
                # seen any of our training sets.
                tasks_on_entry['Polling.RxEQ'] = [
                    ts2_seen      .eq(0)
                ]

                # Our role in this state is to both send TSEQs to enable equalizer training, and to
                # train our own equalizer. We'll request that our frontend train its equalizer, where
                # possible.
                #m.d.comb += self.train_equalizer.eq(1)

                # Continuously send TSEQs; these are used to perform receiver equalization training.
                m.d.comb += self.send_tseq_burst.eq(1)


                # Once we've sent a full burst of 35536 TSEQs, we can begin our link training handshake.
                with m.If(self.ts_burst_complete):
                    transition_to_state("Polling.Active")


            # Polling.Active -- we've now exchanged our initial test sequences, and we're ready to
            # begin exchaning our core training sequences. We'll start sending TS1, and let the PHY handle
            # link training until it reliably the same thing from the host. [USB 3.2r1: 7.5.4.8]
            with m.State("Polling.Active"):
                handle_warm_resets()

                # Constantly send TS1s; which indicate that we're in link training, but haven't yet
                # seen enough TS1s to move forward with training.
                m.d.comb += self.send_ts1_burst.eq(1)

                # If we don't achieve link training within 12mS, we'll assume that we've lost our
                # link partner. We'll start our process again from the beginning.
                transition_on_timeout(12e-3, to="Rx.Detect.Active")

                # Once our ordered set module reports a long enough burst of seen training sets
                # we're satisfied with our link training. However, we don't want to stop sending
                # training data until we're sure our link partner has completed training; so we'll
                # move to Polling.Configuration to await completion of the other side.
                with m.If(self.ts1_detected | self.ts2_detected):
                    m.d.ss += self.invert_rx_polarity.eq(0),
                    transition_to_state("Polling.Configuration")


                # If we see a long enough burst of -inverted- training sets, we're also satisfied
                # with our link training; but we know that the receive differential pair is inverted.
                # We'll continue, but ask our physical layer to invert our received data.
                with m.If(self.inverted_ts1_detected):
                    m.d.ss += self.invert_rx_polarity.eq(1),
                    transition_to_state("Polling.Configuration")


            # Polling.Configuration -- we're now satisfied with our link training; we'll need to communicate
            # this to the other side, and wait for the other side to advertise the same. [USB3.2r1; 7.5.4.9]
            with m.State("Polling.Configuration"):
                handle_warm_resets()

                # Constantly send TS2s, which both allow the other side to continue link training and
                # advertise that our side has completed link training itself.
                m.d.comb += self.send_ts2_burst.eq(1)

                # If we don't achieve link training within 12mS, we'll assume that we've lost our
                # link partner. We'll start our process again from the beginning.
                transition_on_timeout(12e-3, to="Rx.Detect.Active")

                # If we've finished sending the requisite amount of TS2s and we've seen TS2s from the
                # other side, we know that both sides are finished with the core link training.
                # Move on to our final
                with m.If(self.ts_burst_complete & ts2_seen):
                    transition_to_state("Polling.Idle")


            # Polling.Idle -- we've now finished link training, and we're ready to move on to real
            # communications. We'll perform one final sanity check, and then move to our next state.
            # [USB3.2r1: 7.5.4.10]
            with m.State("Polling.Idle"):

                # From this state onward, we have an active link, and we can thus enable data scrambling.
                m.d.comb += self.enable_scrambling.eq(1)

                # Theoretically, in this state, we'd decide between U0, Loopback, or Hot Reset,
                # based on the contents of the most recent TS2 we'd received.
                #
                # For now, we don't support Loopback or Hot Reset, so we'll just always target U0.

                # As one final sanity check, we'll require a proper period of Logical Idle to be
                # detected before we move to our next state. Since Logical Idle signals are scrambled,
                # this helps to ensure that both sides of the link have synchronized scrambler state.
                with m.If(self.logical_idle_detected):
                    transition_to_state("U0")

                # If we don't see that logical idle within 2ms, something's gone wrong. We'll need to
                # start our connection process from the beginning.
                transition_on_timeout(2e-3, to="Rx.Detect.Reset")


            # U0 -- our primary active USB state, in which we've completed link bringup and now are
            # performing normal USB3 operations.
            with m.State("U0"):
                m.d.comb += [

                    # We're now ready for normal operation -- we'll mark our link as ready,
                    # and keep our normal scrambling enabled.
                    self.enable_scrambling  .eq(1),
                    self.link_ready         .eq(1)
                ]

                # TODO: handle the various cases for leaving U0


            # Compliance -- we've failed link training in such a way as to believe we're in the
            # middle of a compliance test / validation (lucky us!).
            with m.State("Compliance"):

                # We don't currently handle Compliance properly. In this case, this message refers
                # to the Compliance state, but this also makes us non-compliant, so this statement
                # has an especially appropriate double meaning.
                #
                # We'll throw our hands up in despair and re-try link training.
                # Maybe this time it'll work.
                transition_to_state("Rx.Detect.Reset")



        return m
