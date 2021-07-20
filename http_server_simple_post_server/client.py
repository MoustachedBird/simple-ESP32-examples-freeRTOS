#!/usr/bin/env python
#
# Copyright 2018 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
from __future__ import unicode_literals
from builtins import str
import http.client
import argparse


from tiny_test_fw import Utility


def verbose_print(verbosity, *args):
    if (verbosity):
        Utility.console_log(''.join(str(elems) for elems in args))


def test_val(text, expected, received):
    if expected != received:
        Utility.console_log(" Fail!")
        Utility.console_log("  [reason] " + text + ":")
        Utility.console_log("        expected: " + str(expected))
        Utility.console_log("        received: " + str(received))
        return False
    return True


def test_post_handler(ip, port, msg, verbosity=False):
    verbose_print(verbosity, "========  POST HANDLER TEST ============")
    # Establish HTTP connection
    verbose_print(verbosity, "Connecting to => " + ip + ":" + port)
    sess = http.client.HTTPConnection(ip + ":" + port, timeout=15)

    # POST message to /echo and get back response
    sess.request("POST", url="/echo", body=msg)
    resp = sess.getresponse()
    resp_data = resp.read().decode()
    verbose_print(verbosity, "Server response to POST /echo (" + msg + ")")
    verbose_print(verbosity, "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
    verbose_print(verbosity, resp_data)
    verbose_print(verbosity, "========================================\n")

    # Close HTTP connection
    sess.close()
    return test_val("Response mismatch", msg, resp_data)



if __name__ == '__main__':
    # Configure argument parser
    parser = argparse.ArgumentParser(description='Run HTTPd Test')
    parser.add_argument('IP',   metavar='IP',      type=str, help='Server IP')
    parser.add_argument('port', metavar='port',    type=str, help='Server port')
    parser.add_argument('msg',  metavar='message', type=str, help='Message to be sent to server')
    args = vars(parser.parse_args())

    # Get arguments
    ip   = args['IP']
    port = args['port']
    msg  = args['msg']

    if not (
        #test_get_handler(ip, port, True) and
        #test_put_handler(ip, port, True) and
        test_post_handler(ip, port, msg, True)
    ):
        Utility.console_log("Failed!")
