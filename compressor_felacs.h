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

//miembros p�blicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//m�todos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//m�todo constructor 1.
	compressor_felacs();

	//-------------------------------------------------------------------------------------
	//m�todo constructor 2.
	compressor_felacs(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//m�todo destructor.
	~compressor_felacs();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//m�todos para FELACS.

	//-------------------------------------------------------------------------------------
	//m�todo de inicializaci�n.
	void initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//m�todo de compresi�n FELACS.
	int compressorFELACS(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//m�todo de descompresi�n FELACS.
	int decompressorFELACS(unsigned char *_stream_com_data, int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "di".
	int getDiFELACS(int _delta, int _xi_1, int _bits_resolution_data);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "delta".
	int getDelta(int _di, int _xi_1, int _bits_resolution_data);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "k".
	int getK(int _D, int _j);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener c�digo Golomb-Rice.
	void getGolombRiceCode(int _k, int _delta, int &_output_code, int &_busy_bits_code);

	//-------------------------------------------------------------------------------------
	//m�todo para limpiar "deltas".
	void cleanDeltas(void);

	//-------------------------------------------------------------------------------------
	//m�todo para limpiar buffers.
	void cleanBuffersFELACS();
};

