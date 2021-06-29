"""Command line interface for the Zephyr BioHarness LSL integration."""

import logging
import datetime
import asyncio
import argparse

# import pylsl

from core import BioHarness
from core.protocol import *

logger = logging.getLogger(__name__)


def add_manufacturer(desc):
    """Add manufacturer into to a stream's desc"""
    acq = desc.append_child('acquisition')
    acq.append_child_value('manufacturer', 'Medtronic')
    acq.append_child_value('model', 'Zephyr BioHarness')


"""
ENABLE FUNCTIONS - START
The following functions are used to enable the output of specific
metrics. The "on_****" functions within them are where the data
are output. Any calls of "outlet.push_chunk()" are reminants of
the lab streaming layer, but give insight to which data from the
msg were broadcasted.
"""

# noinspection PyUnusedLocal
async def enable_ecg(link, nameprefix, idprefix, **kwargs):
    """Enable the ECG data stream. This is the raw ECG waveform."""
    def on_ecg(msg):
        print(msg)

    await link.toggle_ecg(on_ecg)


# noinspection PyUnusedLocal
async def enable_respiration(link, nameprefix, idprefix, **kwargs):
    """Enable the respiration data stream. This is the raw respiration (chest
    expansion) waveform."""

    def on_breathing(msg):
#         INSERT DESIRED OUTPUT FROM msg
#         outlet.push_chunk([[v] for v in msg.waveform])

    await link.toggle_breathing(on_breathing)


# noinspection PyUnusedLocal
async def enable_accel100mg(link, nameprefix, idprefix, **kwargs):
    """Enable the accelerometer data stream. This is a 3-channel stream in units
    of 1 g (earth gravity)."""

    def on_accel100mg(msg):
#         INSERT DESIRED OUTPUT FROM msg        
#         outlet.push_chunk([[x, y, z] for x, y, z in zip(msg.accel_x, msg.accel_y, msg.accel_z)])

    await link.toggle_accel100mg(on_accel100mg)


# noinspection PyUnusedLocal
async def enable_accel(link, nameprefix, idprefix, **kwargs):
    """Enable the regular accelerometer data stream. This is a 3-channel stream
    with slightly higher res than accel100mg (I believe around 2x), but """

    def on_accel(msg):
#         INSERT DESIRED OUTPUT FROM msg
#         outlet.push_chunk([[x, y, z] for x, y, z in zip(msg.accel_x, msg.accel_y, msg.accel_z)])

    await link.toggle_accel(on_accel)


# noinspection PyUnusedLocal
async def enable_rtor(link, nameprefix, idprefix, **kwargs):
    """Enable the RR interval data stream. This has the interval between the
    most recent two ECG R-waves, in ms (held constant until the next R-peak),
    and the sign of the reading alternates with each new R peak."""

    def on_rtor(msg):
#         INSERT DESIRED OUTPUT FROM  msg
#         outlet.push_chunk([[v] for v in msg.waveform])

    await link.toggle_rtor(on_rtor)


async def enable_events(link, nameprefix, idprefix, **kwargs):
    """Enable the events data stream. This has a few system events like button
    pressed, battery low, worn status changed."""

    def on_event(msg):
        if kwargs.get('localtime', '1') == '1':
            stamp = datetime.datetime.fromtimestamp(msg.stamp)
        else:
            stamp = datetime.datetime.utcfromtimestamp(msg.stamp)
        timestr = stamp.strftime('%Y-%m-%d %H:%M:%S')
        event_str = f'{msg.event_string}/{msg.event_data}@{timestr}'
#         outlet.push_sample([event_str])
        logger.debug(f'event detected: {event_str}')

    await link.toggle_events(on_event)


# noinspection PyUnusedLocal
async def enable_summary(link, nameprefix, idprefix, **kwargs):
    """Enable the summary data stream. This has most of the derived data
    channels in it."""

    def on_summary(msg):
        content = msg.as_dict()
#         INSERT DESIRED OUTPUT FROM "content"

    await link.toggle_summary(on_summary)


# noinspection PyUnusedLocal
async def enable_general(link, nameprefix, idprefix, **kwargs):
    """Enable the general data stream. This has summary metrics, but fewer than
    the summary stream, plus a handful of less-useful channels."""

    def on_general(msg):
        content = msg.as_dict()
#         INSERT DESIRED OUTPUT FROM "content"

    await link.toggle_general(on_general)


# map of functions that enable various streams and hook in the respective handlers
enablers = {
    'ecg': enable_ecg,
    'respiration': enable_respiration,
    'accel100mg': enable_accel100mg,
    'accel': enable_accel,
    'rtor': enable_rtor,
    'events': enable_events,
    'summary': enable_summary,
    'general': enable_general,
}

"""
ENABLE FUNCTIONS - END
"""

# our BioHarness link
link = None


async def init():
    global link
    try:
        # parse args from input
        p = argparse.ArgumentParser(
            description='Stream data from the Zephyr BioHarness.')
        p.add_argument('--address', help="Bluetooth MAC address of the device "
                                         "to use (autodiscover if not given).",
                       default='')
        p.add_argument('--port', help='Bluetooth port of the device (rarely '
                                      'used).',
                       default=1)
        p.add_argument('--stream', help='Comma-separated list of data to stream (no spaces).'
                                        'Note that using unnecessary streams will '
                                        'likely drain the battery faster.',
                       default=','.join(enablers.keys()))
        p.add_argument('--loglevel', help="Logging level (DEBUG, INFO, WARN, ERROR).",
                       default='INFO', choices=['DEBUG', 'INFO', 'WARN', 'ERROR'])
        p.add_argument('--streamprefix', help='Stream name prefix. This is pre-pended '
                                              'to the name of all LSL streams.',
                       default='Zephyr')
        p.add_argument('--timeout', help='Command timeout. If a command takes longer '
                                         'than this many seconds to succeed or fail, '
                                         'an error is raised and the app exits.',
                       default=20)
        p.add_argument('--localtime', help="Whether event time stamps are in "
                                           "local time (otherwise UTC is assumed).",
                       default='1', choices=['0', '1'])
        args = p.parse_args()

        # set up logging
        logging.basicConfig(level=logging.getLevelName(args.loglevel),
                            format='%(asctime)s %(levelname)s: %(message)s')
        logger.info("starting up...")

        # sanity checking
        modalities = args.stream.split(',')
        unknown = set(modalities) - set(enablers.keys())
        if unknown:
            raise ValueError(f"Unknown modalities to stream: {unknown}")

        # connect to bioharness
#         args.address = "CC:78:AB:86:0E:28" # in case you want to hard code the mac address
        link = BioHarness(args.address, port=int(args.port), timeout=args.timeout)
        infos = await link.get_infos()
        info_str = '\n'.join([f' * {k}: {v}' for k, v in infos.items()])
        logger.info(f"Device info is:\n{info_str}")
        id_prefix = infos['serial']

        # enable various kinds of streams and install handlers
        logger.info("Enabling streams...")
        for mod in modalities:
            logger.info(f"  enabling {mod}...")
            enabler = enablers[mod]
            await enabler(link, nameprefix=args.streamprefix,
                          idprefix=id_prefix, **vars(args))

        logger.info('Now streaming...')

    except SystemExit:
        asyncio.get_event_loop().stop()
    except TimeoutError as e:
        logger.error(f"Operation timed out: {e}")
        asyncio.get_event_loop().stop()
    except Exception as e:
        logger.exception(e)
        asyncio.get_event_loop().stop()

if __name__ == "__main__":
    asyncio.ensure_future(init())
    loop = asyncio.get_event_loop()
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.info("Ctrl-C pressed.")
    finally:
        if link:
            # noinspection PyUnresolvedReferences
            link.shutdown()
        loop.close()

