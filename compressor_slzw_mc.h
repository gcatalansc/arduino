#pragma once

//inclusiones.
#include "compressor_sn.h"
#include "def_slzw.h"
	
//clase para compresor SLZW_MC.
class compressor_slzw_mc : compressor_sn
{

//miembros privados.
private:

	//buffers.
	unsigned char lzw_output_file_buffer[LZW_BUFFER_SIZE];
	unsigned char write_buffer[BLOCK_SIZE];

	//Holds the dictionary.
	unsigned char compression_chars[(MAX_DICT_ENTRIES + 1) * sizeof(Dict_node)];

//miembros públicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//métodos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//método constructor.
	compressor_slzw_mc();

	//-------------------------------------------------------------------------------------
	//método destructor.
	~compressor_slzw_mc();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//métodos para SLZW_MC.

	//-------------------------------------------------------------------------------------
	//función de compresión S-LZW MC.
	int compressorSLZW_MC(int *_data_in, int _samples_amount, unsigned char *_stream_com_data);

	//-------------------------------------------------------------------------------------
	//función de descompresión S-LZW MC.
	int decompressorSLZW_MC(int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//función para inicializar diccionario.
	void initDictionary();

	//-------------------------------------------------------------------------------------
	//función para inicializar diccionario para descompresión.
	void initDecompDictionary();

	//-------------------------------------------------------------------------------------
	//función para limpiar buffer de datos comprimidos.
	int cleanBuffersSLZW();
};
