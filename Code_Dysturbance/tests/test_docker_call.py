#!/usr/bin/env python3

"""
@file test_docker_call.py
@author Anthony Remazeilles
@brief check the computation runs as expected

Copyright (C) 2019 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).

"""

import os
import io
import unittest
import tempfile
import logging
import sys


class DockerCallTest(unittest.TestCase):
    """gather program tests
    """

    DOCKER_IMAGE = "pi_csic_docker_image"

    def setUp(self):
        """Common initialization operations
        """

        self.log = logging.getLogger("test_log")

        self.log.debug("Setting up the test")

        self.log.debug("Testing image: {}".format(self.DOCKER_IMAGE))
        rel_path = os.path.dirname(__file__)

        self.input_data_path = os.path.abspath(os.getcwd() + "/" + rel_path + "/data/input")

        self.output_groundtruth_path = os.path.abspath(os.getcwd() + "/" + rel_path + "/data/output")

        self.log.debug("Input data in: {}".format(self.input_data_path))

        self.output_data_path = tempfile.mkdtemp()
        os.chmod(self.output_data_path, 0o777)

        # preparing the generation command
        self.command = "docker run --rm -v {}:/in -v {}:/out {} ".format(self.input_data_path,
                                                                         self.output_data_path,
                                                                         self.DOCKER_IMAGE)

        self.command += "./run_pi /in/subject_10_trial_01.csv /in/subject_10_anthropometry.yaml /out"

        self.log.debug("Command generated: \n{}".format(self.command))

    def test_call_docker(self):
        """test the docker component with stored input and output

        """

        self.log.info("Launching docker command")
        # TODO how to catch the result of the command (error or success)
        os.system(self.command)

        self.log.info("Docker command launched")

        # check generated files
        output_files = os.listdir(self.output_data_path)
        output_files_expected = os.listdir(self.output_groundtruth_path)

        self.assertCountEqual(output_files, output_files_expected)

        # Check the content of each file

        for filename in output_files:
            self.log.debug("comparing file: {}".format(filename))

            file_generated = self.output_data_path + "/" + filename

            lines_generated = list()
            with open(file_generated) as f:
                for line in f:
                    lines_generated.append(line)

            file_groundtruth = self.output_groundtruth_path + "/" + filename

            lines_groundtruth = list()
            with open(file_groundtruth) as f:
                for line in f:
                    lines_groundtruth.append(line)

            # print("Comparing:\n{}\n with \n{}".format(lines_generated, lines_groundtruth))
            self.assertListEqual(lines_generated, lines_groundtruth)

        self.log.info("Test completed")


if __name__ == '__main__':
    print("test_docker_call -- testing image: {}".format(os.environ.get('DOCKER_IMAGE')))

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

    DockerCallTest.DOCKER_IMAGE = os.environ.get('DOCKER_IMAGE', DockerCallTest.DOCKER_IMAGE)
    # TODO using https://stackoverflow.com/questions/11380413/python-unittest-passing-arguments
    # but it is mentioned as not preferrable.
    unittest.main()
