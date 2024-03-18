# Open AFE
OpenAFE is a pioneering open-source initiative focused on Electrochemical Analog Front Ends (AFE), designed to democratize access to electrochemical AFE technology. This initiative has yielded an Arduino-compatible shield and a C-compatible library, both readily adaptable for use with MCUs beyond ATMEGAs.

Here is a Cyclic Voltammetry done with OpenAFE library and the shield, the graph was acquired using [OpenAFE_PythonPlotter](https://github.com/ig-66/OpenAFE_PythonPlotter):

![plotterexample](https://github.com/moduhub/openafe/blob/main/resources/images/CV_salt_water.png)

# How to Use with Python

To do that follow the instruction at [OpenAFE_PythonPlotter](https://github.com/ig-66/OpenAFE_PythonPlotter). The [OpenAFE_Comm](https://github.com/ig-66/openAFEComm) is also required.

# How to Compile for Other MCUs
Requirements:
* SPI clock speed: from 1 MHz up to 16 MHz.

Add the MCU specific commands in the in the wrappers, located in `src/openafe_wrapper/openafe_wrapper.c`.

If necessary take a look at the zephyr branch for as an example.

# License
Copyright 2023 ModuHub Tecnologia Ltda

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies. 

THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.