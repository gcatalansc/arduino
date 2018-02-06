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

//miembros p�blicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//m�todos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//m�todo constructor.
	compressor_slzw_mc();

	//-------------------------------------------------------------------------------------
	//m�todo destructor.
	~compressor_slzw_mc();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//m�todos para SLZW_MC.

	//-------------------------------------------------------------------------------------
	//funci�n de compresi�n S-LZW MC.
	int compressorSLZW_MC(int *_data_in, int _samples_amount, unsigned char *_stream_com_data);

	//-------------------------------------------------------------------------------------
	//funci�n de descompresi�n S-LZW MC.
	int decompressorSLZW_MC(int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//funci�n para inicializar diccionario.
	void initDictionary();

	//-------------------------------------------------------------------------------------
	//funci�n para inicializar diccionario para descompresi�n.
	void initDecompDictionary();

	//-------------------------------------------------------------------------------------
	//funci�n para limpiar buffer de datos comprimidos.
	int cleanBuffersSLZW();
};
