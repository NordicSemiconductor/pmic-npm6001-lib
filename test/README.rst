nPM6001 lib test hardware mapping
#################################

.. list-table::
   :header-rows: 1
   :align: center

   * - Function
     - nPM6001
     - nRF5340-DK pin
   * - nPM6001 power supply
     - P1 (VIN)
     - -
   * - Reset signal
     - Header P15, READY
     - Header P1, RESET
   * - Two-Wire, SDA
     - Header P7, SDA
     - Header P4, GPIO P1.02 (Arduino SDA)
   * - Two-Wire, SCL
     - Header P7, SCL
     - Header P4, GPIO P1.03 (Arduino SCL)
   * - Two-Wire, reference voltage
     - Header P7, VCC
     - Header P1, VDD
   * - BUCK0 measurement
     - BUCK0 (DCDC0)
     - Header P2, GPIO P0.04
   * - BUCK1 measurement
     - BUCK1 (DCDC1)
     - Header P2, GPIO P0.05
   * - BUCK2 measurement
     - BUCK2 (DCDC2)
     - Header P2, GPIO P0.06
   * - BUCK3 measurement
     - BUCK3 (DCDC3)
     - Header P2, GPIO P0.07
   * - LDO0 measurement
     - VLDO0
     - Header P2, GPIO P0.25
   * - LDO1 measurement
     - VLDO1
     - Header P2, GPIO P0.26
