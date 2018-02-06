
//inclusiones.
#include "compressor_ldc.h"
#include <Arduino.h>

//-------------------------------------------------------------------------------------
//m�todo constructor 1.
compressor_ldc::compressor_ldc()
{
	
}

//-------------------------------------------------------------------------------------
//m�todo constructor 2.
compressor_ldc::compressor_ldc(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size, int *_dataset, int _length_dataset)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resoluci�n en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//m�ximo tama�o del buffer de compresi�n de datos.
	
	//buffers.
	this->di_ldc				= new int[_max_samples_amount - 1];							//diferencias entre xi y xi_1 para LDC;
	this->diset_ldc				= new int[_length_dataset];									//diferencias entre xi y xi_1 para set de datos del algoritmo LDC.
	this->length_dataset		= _length_dataset;											//largo del vector dataset.
	this->length_fdi			= 0;														//cantidad de "di".	

	//inicializaci�n.
	this->cleanBuffersLDC();
	this->iniTableLDC(this->tree_huffman_code, _dataset);
}

//-------------------------------------------------------------------------------------
//m�todo destructor.
compressor_ldc::~compressor_ldc()
{

}

//-------------------------------------------------------------------------------------
//m�todo inicializaci�n.
void compressor_ldc::initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size, int *_dataset, int _length_dataset)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resoluci�n en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//m�ximo tama�o del buffer de compresi�n de datos.
	
	//buffers.
	this->di_ldc				= new int[_max_samples_amount - 1];							//diferencias entre xi y xi_1 para LDC;
	this->diset_ldc				= new int[_length_dataset];									//diferencias entre xi y xi_1 para set de datos del algoritmo LDC.
	this->length_dataset		= _length_dataset;											//largo del vector dataset.
	this->length_fdi			= 0;														//cantidad de "di".	

	//inicializaci�n.
	this->cleanBuffersLDC();
	this->iniTableLDC(this->tree_huffman_code, _dataset);
}

//-------------------------------------------------------------------------------------
//m�todo de compresi�n LDC.
int compressor_ldc::compressorLDC(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//variables auxiliares.
	int code			= 0;
	int busy_bits_code	= 0;

	//bucle para obtener los "di".
	for (int i1 = 0; i1 < (_samples_amount - 1); i1++)
	{
		//obtenci�n de "di".
		this->di_ldc[i1] = _data_in[i1 + 1] - _data_in[i1];
	}

	//seteo de primer dato de entrada sin comprimir.
	joinBits(_data_in[0], _bits_resolution_data, _stream_com_data, _stream_index_available, _stream_available_bits);
	
	//bucle para obtener c�digo huffman de cada "di".
	for (int i1 = 0; i1 < (_samples_amount - 1); i1++)
	{
		//obtenci�n del c�digo.
		getHuffmanCode(this->di_ldc[i1], code, busy_bits_code);
		
		//seteo de c�digo en buffer de datos comprimidos.
		joinBits(code, busy_bits_code, _stream_com_data, _stream_index_available, _stream_available_bits);
	}

	//retorno de la cantidad de bytes resultantes de la compresi�n.
	if (_stream_available_bits == 8)	return _stream_index_available;
	else								return _stream_index_available + 1;
}

//-------------------------------------------------------------------------------------
//m�todo de descompresi�n LDC.
int compressor_ldc::decompressorLDC(unsigned char *_stream_com_data, int _samples_amount, int *_data_out)
{
	//variables auxiliares.	
	int i1						= 0;
	int code					= 0;
	int di_aux					= 0;
	int di_index				= 0;
	int	bit_position			= 0;
	int bits_counter			= 0;
	int	index_amount			= int(floor((this->bits_resolution_data - 8) / 8.0f));
	int	remaining_bits_amount	= 0;
	int	temp					= 0;

	//seteo primeros 8 bits de "data_out[0]".
	this->joinBits(_stream_com_data[0], 8, _data_out[0]);
	
	//bucle para setear los bytes restantes de "data_out[0]".
	for (i1 = 1; i1 < index_amount; i1++)
	{
		//seteo de bytes restantes de "data_out[0]".
		this->joinBits(_stream_com_data[i1], 8, _data_out[0]);
	}
	
	//bits remanentes de "data_out[0]".
	remaining_bits_amount = this->bits_resolution_data - (8 * (index_amount + 1));
	
	//si existen bits restantes de "data_out[0]", entonces.
	if (remaining_bits_amount > 0)
	{
		//seteo de los bits restantes de "data_out[0]".
		temp	= _stream_com_data[i1] >> (8 - remaining_bits_amount);
		this->joinBits(temp, remaining_bits_amount, _data_out[0]);
	}
	
	//actualizaci�n de variables auxiliares.
	bit_position = 7 - remaining_bits_amount;	

	//bucle para obtener "di".
	do
	{
		//obtenci�n de "di".
		if ((_stream_com_data[i1] & (1 << bit_position)) == 0)	code = code << 1;
		else													code = (code << 1) | 1;

		//decremento de "bit_position".
		--bit_position;

		//si "bit_position" alcanza su m�nimo valor, entonces.
		if (bit_position == -1)
		{
			//actualizaciones.
			bit_position = 7;
			++i1;
		}

		//incremento del contador de bits.
		++bits_counter;

		//obtenci�n de "di".
		di_aux = this->getDiLDC(code, bits_counter);
		
		//si se ha obtenido "di", entonces.
		if (di_aux != -1)
		{
			//seteo de "di".
			this->di_ldc[di_index]	= di_aux;
			
			//incremento del �ndice de "di".
			++di_index;

			//reset de variables.
			bits_counter		= 0;
			code				= 0;	

			//si es el �ltimo dato, entonces.
			if ((di_index + 1) == _samples_amount) break;
		}

	} while (true);

	//bucle para obtener datos de salida.
	for (int i2 = 1; i2 < _samples_amount; i2++)
	{
		//obtenci�n de "data_out[i2]".
		_data_out[i2] = _data_out[i2 - 1] + this->di_ldc[i2 - 1];
	}

	//limpieza de buffers.
	this->cleanBuffersLDC();

	//retorno de la cantidad de bytes resultantes de la descompresi�n.
	return (_samples_amount * ceil(float(this->bits_resolution_data) / 8.f));
}

//-------------------------------------------------------------------------------------
//m�todo para obtener la cantidad de "di" diferentes.
int compressor_ldc::getDiCounter()
{
	//contador de "di".
	int		di_counter	= 1;
	bool	check_di	= false;

	//bucle para obtener repeticiones de "diset_ldc[i1]".
	for (int i1 = 1; i1 < this->length_dataset; i1++)
	{
		//reset "check_di".
		check_di = false;

		//bucle para cotejar "diset_ldc[i1]" cada elemento anterior.
		for (int i2 = 0; i2 < i1; i2++)
		{
			//si hay un elemento anterior, entonces.
			if (this->diset_ldc[i1] == this->diset_ldc[i1])	check_di = true;
		}

		//si "di" no est� repetido, entonces.
		if (check_di == false)	++di_counter;
	}

	//retorno de la cantidad de "di" sin repeticiones.
	return di_counter;
}

//-------------------------------------------------------------------------------------
//m�todo para setear los "di_set".
void compressor_ldc::setDi(int *_dataset_in)
{
	//bucle para obtener los "di_set".
	for (int i1 = 0; i1 < (this->length_dataset - 1); i1++)
	{
		//obtenci�n de "di_set".
		this->diset_ldc[i1] = _dataset_in[i1 + 1] - _dataset_in[i1];
	}

	//obtenci�n de la cantidad de "di" sin repetici�n.
	this->length_fdi = this->getDiCounter();
	
	//inicializaci�n de "table" y "fdi".
	this->table_ldc	= new int*[this->length_fdi];													
	this->fdi		= new int*[this->length_fdi];	

	//bucle para inicializar posiciones de "table" y "fdi".
	for (int i1 = 0; i1 < this->length_fdi; i1++)
	{
		//inicializaci�n de posiciones de "table" y "fdi".
		this->table_ldc[i1] = new int[3];
		this->fdi[i1]		= new int[2];
	}
}

//-------------------------------------------------------------------------------------
//m�todo para obtener la frecuencia ordenadas de menor a mayor de cada "diset_ldc".
void compressor_ldc::setFDI()
{
	//variables auxiliares.
	int fdi_index = -1;
	
	//bucle para cada "diset_ldc".
	for (int i1 = 0; i1 < (this->length_dataset - 1); i1++)
	{
		//si "diset_ldc" no est� en buffer "fdi", entonces.
		if (!this->checkDiOnFdi(this->diset_ldc[i1]))
		{
			//incremento del contador de "diset_ldc".
			++fdi_index;
			
			//seteo de "di" en "fdi".
			this->fdi[fdi_index][0] = this->diset_ldc[i1];
			this->fdi[fdi_index][1] = 0;
			
			//bucle para obtener la frecuencia de los "diset_ldc".
			for (int i2 = 0; i2 < (this->length_dataset - 1); i2++)
			{
				//si "diset_ldc[i1]" corresponde con "diset_ldc[i2]". 
				if (this->diset_ldc[i1] == this->diset_ldc[i2])
				{
					//incremento de la frecuencia de "diset_ldc".
					++this->fdi[fdi_index][1];
				}
			}	
		}		
	}	

	//ordenamiento de menor a mayor del buffer "fdi".
	this->orderFDI();
}

//-------------------------------------------------------------------------------------
//m�todo para ordenadar las frecuencias de menor a mayor de cada "di_set".
void compressor_ldc::orderFDI(void)
{
	//variables auxiliares.
	int		temp_0		= 0;
	int		temp_1		= 0;
	bool	check_order	= false;
	
	//ordenamiento de menor a mayor del buffer "fdi".
	do
	{
		//inicializaci�n de variable auxiliar de checkeo de intercambio.
		check_order = false;

		//bucle para ordenar de menor a mayor buffer "fdi".
		for (int i1 = 0; i1 < this->length_fdi - 1; i1++)
		{
			//si el valor de "fdi" actual es mayor que el siguiente, entonces.
			if (this->fdi[i1][1] > this->fdi[i1 + 1][1])
			{
				//respaldo de valor actual.
				temp_0			= this->fdi[i1][0];
				temp_1			= this->fdi[i1][1];

				//intercambio de posiciones.
				this->fdi[i1][0]		= this->fdi[i1 + 1][0];
				this->fdi[i1][1]		= this->fdi[i1 + 1][1];
				this->fdi[i1 + 1][0]	= temp_0;
				this->fdi[i1 + 1][1]	= temp_1;

				//seteo de variable auxiliar de checkeo de intercambio.
				check_order = true;
			}
		}		

	} while (check_order);
}

//-------------------------------------------------------------------------------------
//m�todo para comprobar si existe "di_set" en buffer "fdi".
bool compressor_ldc::checkDiOnFdi(int _di_set)
{
	//bucle para checkear "di" en buffer "fdi".
	for (int i1 = 0; i1 < (this->length_fdi + 1); i1++)
	{
		//si existe, entonces retornar verdadero.
		if (_di_set == this->fdi[i1][0]) return true;
	}

	//si no existe, entonces retornar falso.
	return false;
}

//-------------------------------------------------------------------------------------
//m�todo para inicializar tabla.
void compressor_ldc::iniTableLDC(treeHuffmanCode &_tree_huffman_code, int *_dataset)
{
	//�ndice para tabla.
	int index_table = -1;

	//seteo de los "di".
	this->setDi(_dataset);	

	//seteo de "fdi".
	this->setFDI();
	
	//bucle para inicializar tabla.
	for (int i1 = 0; i1 < this->length_fdi; i1++)
	{
		//inicializaci�n.
		this->table_ldc[i1][0] = this->fdi[i1][0];
		this->table_ldc[i1][1] = 0;
		this->table_ldc[i1][2] = 0;
	}
	
	//obtenci�n del �rbol para obtener c�digo huffman.
	this->getHuffmanTree(_tree_huffman_code);

	//creaci�n de tabla con c�digos huffman.
	this->createTableLDC(_tree_huffman_code, index_table);
}

//-------------------------------------------------------------------------------------
//m�todo para crear tabla de c�digos huffman.
void compressor_ldc::createTableLDC(treeHuffmanCode &_node, int &_index_table)
{
	//c�digo auxiliar.
	int aux_code	= this->table_ldc[_index_table][1];
	int counter		= this->table_ldc[_index_table][2];
	
	//si "fdi_amount" es cero, entonces.
	if (this->length_fdi == 1)
	{
		//seteo de "di" y "0" en código.
		this->table_ldc[_index_table][0] = _node.di;
		this->table_ldc[_index_table][1] = 0;
		this->table_ldc[_index_table][2] = 1;
	}
	
	//si "fdi_amount" es distinto de cero, entonces.
	else
	{
		//si se ha llegado a un "di" en la izquierda, entonces.
		if (_node.left_tree->left_tree == NULL)
		{
			//seteo de "di" y "0" en c�digo.
			this->table_ldc[_index_table][0] = _node.left_tree->di;
			this->table_ldc[_index_table][1] = aux_code << 1;
			this->table_ldc[_index_table][2] = counter + 1;

			//incremento del �ndice de tabla.
			++_index_table;
		}

		//si no se ha llegado a un "di" en la izquierda, entonces.
		else
		{
			//seteo de "0" en c�digo.
			this->table_ldc[_index_table][1] = aux_code << 1;
			this->table_ldc[_index_table][2] = counter + 1;

			//se contin�a la creaci�n de la tabla.
			this->createTableLDC(*_node.left_tree, _index_table);
		}
		
		//si se ha llegado a un "di" en la derecha, entonces.
		if (_node.right_tree->right_tree == NULL)
		{
			//seteo de "di" y "1" en c�digo.
			this->table_ldc[_index_table][0] = _node.right_tree->di;
			this->table_ldc[_index_table][1] = (aux_code << 1) | 1;
			this->table_ldc[_index_table][2] = counter + 1;

			//incremento del �ndice de tabla.
			++_index_table;
		}

		//si no se ha llegado a un "di" en la derecha, entonces.
		else
		{
			//seteo de "1" en c�digo.
			this->table_ldc[_index_table][1] = (aux_code << 1) | 1;
			this->table_ldc[_index_table][2] = counter + 1;

			//se contin�a la creaci�n de la tabla.
			this->createTableLDC(*_node.right_tree, _index_table);
		}
	}	
}

//-------------------------------------------------------------------------------------
//m�todo para obtener �rbol de c�digo huffman tipo 1.
void compressor_ldc::getHuffmanTree(treeHuffmanCode &_tree_huffman_code)
{
	//variables auxiliares.
	int				tree_amount		= this->length_fdi;
	treeHuffmanCode	*new_tree		= NULL;
	treeHuffmanCode	*tree_root		= NULL;	
	treeHuffmanCode	*tree_aux		= NULL;
	treeHuffmanCode	*tree_pres_aux	= NULL;
	treeHuffmanCode	*tree_past_aux	= NULL;
	
	//creaci�n inicial de �rboles con "fdi".
	for (int i1 = 0; i1 < tree_amount; i1++)
	{
		//creaci�n de �rboles.
		new_tree = new treeHuffmanCode;

		//si "i1" es igual que "cero" entonces.
		if (i1 == 0)	tree_root			= new_tree;
		else			tree_aux->next_tree	= new_tree;

		//seteo de puntero auxiliar.
		tree_aux				= new_tree;
		
		//seteo del �rbol creado.
		new_tree->di			= this->fdi[i1][0];
		new_tree->sum			= this->fdi[i1][1];
		new_tree->left_tree		= NULL;
		new_tree->right_tree	= NULL;		
		new_tree->next_tree		= NULL;		
	}

	//si hay más de un árbol, entonces.
	if (tree_amount > 1)
	{
		//bucle para setear árbol para obtener código huffman.
		do
		{
			//creación de un nodo con los dos primeros árboles.
			new_tree				= new treeHuffmanCode;
	
			//seteo del árbol creado.
			new_tree->sum			= tree_root->sum + tree_root->next_tree->sum;
			new_tree->left_tree		= tree_root;
			new_tree->right_tree	= tree_root->next_tree;
			new_tree->next_tree		= NULL;		

			//actualización de la cantidad de árboles.
			--tree_amount;		

			//si hay un solo árbol, entonces.
			if (tree_amount == 1) break;

			//si hay más de un árbol, entonces.
			else 
			{
				//seteo de punteros auxiliares.
				tree_pres_aux = tree_root->next_tree->next_tree;
				tree_past_aux = tree_pres_aux;	

				//bucle para concatenar nuevo nodo.
				do
				{
					//si la suma acumulada de frecuencias del nuevo nodo es menor, entonces.
					if (new_tree->sum < tree_pres_aux->sum)
					{
						//concatenación del nuevo nodo con el siguiente.
						new_tree->next_tree = tree_pres_aux;

						//si los punteros son distintos.
						if (tree_pres_aux != tree_past_aux)
						{
							//concatenación del nuevo nodo con el anterior
							tree_past_aux->next_tree = new_tree;

							//actualización del árbol raiz.
							tree_root = tree_root->next_tree->next_tree;
						}

						//si los punteros son iguales.
						else
						{
							//actualización del árbol raiz.
							tree_root = new_tree;
						}

						//salir del bucle.
						break;
					}

					//si la suma acumulada de frecuencias del nuevo nodo es igual, entonces.
					else if (new_tree->sum == tree_pres_aux->sum)
					{
						//concatenación del nuevo nodo con el siguiente y el anterior.
						new_tree->next_tree			= tree_pres_aux->next_tree;
						tree_pres_aux->next_tree	= new_tree;					

						//actualización del árbol raiz.
						tree_root					= tree_root->next_tree->next_tree;

						//salir del bucle.
						break;
					}

					//si la suma acumulada de frecuencias del nuevo nodo es mayor, entonces.
					else if (new_tree->sum > tree_pres_aux->sum)
					{
						//actualización de punteros auxiliares.
						tree_past_aux = tree_pres_aux;
						tree_pres_aux = tree_pres_aux->next_tree;

						//si se ha llegado al último nodo, entonces.
						if (tree_pres_aux == NULL)
						{
							//concatenación del nuevo nodo con el siguiente.
							tree_past_aux->next_tree	= new_tree;
							new_tree->next_tree			= NULL;

							//actualización del árbol raiz.
							tree_root					= tree_root->next_tree->next_tree;

							//salir del bucle.
							break;
						}
					}

				} while (true);

				//reset de punteros.
				new_tree->left_tree->next_tree	= NULL;
				new_tree->right_tree->next_tree = NULL;	
			}

		} while (true);
	}

	//seteo de la raiz del �rbol creado.
	_tree_huffman_code.di			= new_tree->di;
	_tree_huffman_code.next_tree	= new_tree->next_tree;
	_tree_huffman_code.left_tree	= new_tree->left_tree;
	_tree_huffman_code.right_tree	= new_tree->right_tree;
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "huffman code" en tabla.
void compressor_ldc::getHuffmanCode(int _di, int &_code, int &_busy_bits_code)
{
	//bucle para obtener c�digo huffman de tabla.
	for (int i1 = 0; i1 < this->length_fdi; i1++)
	{
		//si existe correspondencia de "_di", entonces.
		if (_di == this->table_ldc[i1][0])
		{
			//seteo de c�digo y bits ocupados por el c�digo.
			_code			= this->table_ldc[i1][1];
			_busy_bits_code = this->table_ldc[i1][2];
		}
	}
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "di" en tabla.
int compressor_ldc::getDiLDC(int _code, int _busy_bits_code)
{
	//bucle para cotejar "_code" en tabla.
	for (int i1 = 0; i1 < this->length_fdi; i1++)
	{
		//si existe en tabla, entonces.
		if (_code == this->table_ldc[i1][1] && _busy_bits_code == this->table_ldc[i1][2]) return this->table_ldc[i1][0];
	}

	//si no existe en tabla, entonces.
	return -1;
}

//-------------------------------------------------------------------------------------
//método para obtener caontenido den tabla.
int compressor_ldc::getTable(int _row, int _columm)
{
	//retorno del contenido de la tabla.
	return this->table_ldc[_row][_columm];
}

//-------------------------------------------------------------------------------------
//método para obtener cantidad e fdi.
int compressor_ldc::getLength_fdi()
{
	//retorno del largo del fdi.
	return this->length_fdi;
}

//-------------------------------------------------------------------------------------
//método para obtener el máximo valor de una columna de la tabla.
int compressor_ldc::maxTable(int _co)
{
	//inicialización del valor máximo.
	int max = this->table_ldc[0][_co];

	//bucle para hallar valor máximo.
	for (int i1 = 1; i1 < this->length_fdi; i1++)
	{
		//actualización del valor máximo.
		if (max < this->table_ldc[i1][_co]) max = this->table_ldc[i1][_co];
	}

	//retorno del valor máximo.
	return max;
}

//-------------------------------------------------------------------------------------
//m�todo para limpiar buffers. 
void compressor_ldc::cleanBuffersLDC()
{
	//bucle para limpiar buffer.
	for (int i1 = 0; i1 < (this->max_samples_amount - 1); i1++)
	{
		//limpieza.
		this->di_ldc[i1] = 0;
	}

	//bucle para limpiar buffer.
	for (int i1 = 0; i1 < this->max_samples_amount; i1++)
	{
		//limpieza.
		this->diset_ldc[i1] = 0;
	}
}
