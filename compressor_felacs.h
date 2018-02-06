#pragma once

//inclusiones.
#include "compressor_sn.h"

//clase para compresor FELACS.
class compressor_felacs :  compressor_sn
{
//miembros privados.
private:

	//buffers.
	int	*di_felacs;																				//diferencias entre xi y xi_1 para ALDC;
	int	*delta;																					//deltas.

//miembros públicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//métodos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//método constructor 1.
	compressor_felacs();

	//-------------------------------------------------------------------------------------
	//método constructor 2.
	compressor_felacs(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//método destructor.
	~compressor_felacs();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//métodos para FELACS.

	//-------------------------------------------------------------------------------------
	//método de inicialización.
	void initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//método de compresión FELACS.
	int compressorFELACS(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//método de descompresión FELACS.
	int decompressorFELACS(unsigned char *_stream_com_data, int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//método para obtener "di".
	int getDiFELACS(int _delta, int _xi_1, int _bits_resolution_data);

	//-------------------------------------------------------------------------------------
	//método para obtener "delta".
	int getDelta(int _di, int _xi_1, int _bits_resolution_data);

	//-------------------------------------------------------------------------------------
	//método para obtener "k".
	int getK(int _D, int _j);

	//-------------------------------------------------------------------------------------
	//método para obtener código Golomb-Rice.
	void getGolombRiceCode(int _k, int _delta, int &_output_code, int &_busy_bits_code);

	//-------------------------------------------------------------------------------------
	//método para limpiar "deltas".
	void cleanDeltas(void);

	//-------------------------------------------------------------------------------------
	//método para limpiar buffers.
	void cleanBuffersFELACS();
};

