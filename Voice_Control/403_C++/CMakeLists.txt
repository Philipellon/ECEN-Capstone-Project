idf_component_register(SRCS "model_quant_data.cc" "digitalread.c" "main.cpp"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES
                        tflite-micro
                
                    )

