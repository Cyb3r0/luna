#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
#
# Code based on ``litex`` and ``usb3_pipe``.
# SPDX-License-Identifier: BSD-3-Clause
""" Core transmit and receive datapaths. """

from nmigen import *

from ...usb.stream import USBRawSuperSpeedStream
from .gearing      import ReceiverGearbox, TransmitterGearbox, RxWordAligner


class ReceivePostprocessing(Elaboratable):
    """ Receiver post-processing.

    Processes data received from our platform SerDes into data suitable for use in our PHY.

    This handles crossing data from the recovered clock domain into our local one, gearing it
    down to an easily processable data rate, and ensures we're properly word aligned.
    """
    def __init__(self, input_clock_domain="fast", output_clock_domain="ss"):
        self._input_clock_domain  = input_clock_domain
        self._output_clock_domain = output_clock_domain

        #
        # I/O port
        #
        self.align  = Signal()
        self.sink   = USBRawSuperSpeedStream(payload_words=2)
        self.source = USBRawSuperSpeedStream(payload_words=4)

        # Debug signals
        self.alignment_offset = Signal(range(4))


    def elaborate(self, platfrom):
        m = Module()

        #
        # 1:2 Gearing (and clock domain crossing).
        #
        m.submodules.gearing = gearing = ReceiverGearbox(
            input_domain  = self._input_clock_domain,
            output_domain = self._output_clock_domain,
        )
        m.d.comb += gearing.sink.stream_eq(self.sink)

        #
        # Clock tolerance compensation.
        #
        #m.submodules.skip_remover = skip_remover = ReceiverSkipRemover()
        #m.d.comb += skip_remover.sink.stream_eq(gearing.source)

        #
        # Word aligner.
        #
        m.submodules.word_aligner = word_aligner = RxWordAligner()
        m.d.comb += [
            #  Core outputs
            word_aligner.align      .eq(self.align),
            word_aligner.sink       .stream_eq(gearing.source),

            # Debug output.
            self.alignment_offset   .eq(word_aligner.alignment_offset)

        ]

        #
        # Final output.
        #
        m.d.comb += self.source.stream_eq(word_aligner.source)

        return m



class TransmitPreprocessing(Elaboratable):
    """ Transmiter pre-processing.

    Processes data from our PHY, and prepare sit for transmission via our SerDes. This
    gears our data rate up to the rate consumed by the SerDes, and crosses it into our SerDes
     domain.
    """
    def __init__(self, output_clock_domain="fast", input_clock_domain="ss"):
        self._output_domain = output_clock_domain
        self._input_domain  = input_clock_domain

        self.sink   = USBRawSuperSpeedStream(payload_words=4)
        self.source = USBRawSuperSpeedStream(payload_words=2)


    def elaborate(self, platform):
        m = Module()

        #
        # Clock tolerance compensation
        #
        #m.submodules.skip_inserter = skip_inserter = TXSKPInserter()
        #m.d.comb += skip_inserter.sink.stream_eq(self.sink)


        #
        # Output gearing (& clock-domain crossing)
        #
        m.submodules.gearing = gearing = TransmitterGearbox(
            input_domain  = self._input_domain,
            output_domain = self._output_domain
        )
        m.d.comb += gearing.sink.stream_eq(self.sink)
        #self.comb += gearing.sink.stream_eq(skip_inserter.source)

        #
        # Final output
        #
        m.d.comb += self.source.stream_eq(gearing.source)

        return m


