#include "compressor_slzw_mc.h"


//-------------------------------------------------------------------------------------
//método constructor.
compressor_slzw_mc::compressor_slzw_mc()
{
	//bucle para limpiar buffer "this->lzw_output_file_buffer".
	for (int i1 = 0; i1 < LZW_BUFFER_SIZE; i1++)
	{
		//limpieza.
		this->lzw_output_file_buffer[i1] = 0;
	}

	//bucle para limpiar buffer "this->write_buffer".
	for (int i1 = 0; i1 < BLOCK_SIZE; i1++)
	{
		//limpieza.
		this->write_buffer[i1] = 0;
	}
}

//-------------------------------------------------------------------------------------
//método destructor.
compressor_slzw_mc::~compressor_slzw_mc()
{
}

//-------------------------------------------------------------------------------------
//función de compresión S-LZW MC.
int compressor_slzw_mc::compressorSLZW_MC(int *_data_in, int _samples_amount, unsigned char *_stream_com_data)
{
	unsigned char time_to_add, last_char, new_char, alignment = 0, add_to_mc = 0;
	unsigned int size_of_entry = 9, i, current_dict_entry, temp_dict_entry, lzw_output_file_counter, temp_hash, next_dict_entry;
	unsigned long int temp_entry; 
	int bytes_in_data_in = sizeof(int)*_samples_amount; 
	char write_buffer_aux[4];
	Dict_node *current_node, *node;
	

	//bucle para llenar buffer a comprimir.
	for (int i1 = 0; i1 < _samples_amount; i1++)
	{
		//obtención de los bytes de _data_in[i1].
		*((int *)write_buffer_aux) = _data_in[i1];

		//llenado de buffer para su compresión.
		this->write_buffer[i1*4]		= write_buffer_aux[0];
		this->write_buffer[i1*4 + 1]	= write_buffer_aux[1];
		this->write_buffer[i1*4 + 2]	= write_buffer_aux[2];
		this->write_buffer[i1*4 + 3]	= write_buffer_aux[3];
	}
	
	unsigned char mini_dict_hash = MAX_SIZE_OF_MINI_CACHE - 1;
	unsigned char hit_bits;
	
	#ifdef USE_MINICACHE
		// Holds the actual mini cache.
		unsigned int mini_cache_structure[MAX_SIZE_OF_MINI_CACHE];
	#endif
	
	#ifdef STRUCTURED_TRANSFORM
		unsigned int j;
	#endif
	
	node = (Dict_node *) this->compression_chars;
	
	initDictionary();
	next_dict_entry = 256;	
	
	hit_bits = 0;
	
	for (i = MAX_SIZE_OF_MINI_CACHE; i != 0; i=i>>1) {
		hit_bits++;
	}
	
	#ifdef STRUCTURED_TRANSFORM
		// There is probably a smarter way to do this, but this was fast enough for the block size.
		lzw_output_file_counter = 0;
		for (i=0; i<SIZE_OF_READING; i++)
		{
			for (j=i; j < bytes_in_data_in; j+=SIZE_OF_READING)
			{
				this->lzw_output_file_buffer[lzw_output_file_counter++] = this->write_buffer[j];	
			}
		}

		for (int i1 = 0; i1 < bytes_in_data_in; i1++)
		{
			this->write_buffer[i1] = this->lzw_output_file_buffer[i1];
		}

	#endif
	
	// Get first byte and start i at 1	
	last_char = this->write_buffer[0];
	i = 1;
	time_to_add = 0;
	current_node = &(node[last_char]);
	current_dict_entry = last_char;
	
	for (int i1 = 0; i1 < LZW_BUFFER_SIZE; i1++)
	{
		this->lzw_output_file_buffer[i1] = 0;
	}
	
	for (lzw_output_file_counter = 0; lzw_output_file_counter<MAX_SIZE_OF_MINI_CACHE; lzw_output_file_counter++) {
		mini_cache_structure[lzw_output_file_counter] = lzw_output_file_counter;
	}
	
	this->lzw_output_file_buffer[0] = MAX_SIZE_OF_MINI_CACHE;
	lzw_output_file_counter = 1;
	
	for (; i < bytes_in_data_in; i++) {
		new_char = this->write_buffer[i];
			
		if (current_node->longer_string == 0) { // No longer entries in dictionary
			// No strings of this length, so add a deeper entry to the dictionary
			#ifndef RESETABLE_DICTIONARY
			if (next_dict_entry < MAX_DICT_ENTRIES) {
			#endif
			current_node->longer_string = next_dict_entry;
			#ifndef RESETABLE_DICTIONARY
			}
			#endif
			time_to_add = 1;
		}
		else {
			temp_dict_entry = current_node->longer_string;
			current_node = &(node[current_node->longer_string]);
			
			temp_hash = (current_node->next_hash1 << 8) + current_node->next_hash2;
			while ((current_node->entry != new_char) && (temp_hash != 0)) {
				current_node = &(node[temp_hash]);
				temp_dict_entry = temp_hash;
				temp_hash = (current_node->next_hash1 << 8) + current_node->next_hash2;
			}
			
			// Did we find the last letter in the string?
			if ((current_node->entry != new_char) && (temp_hash == 0)) {
				// No, so add a parallel entry to the dictionary
				#ifndef RESETABLE_DICTIONARY
				if (next_dict_entry < MAX_DICT_ENTRIES)
				{
				#endif
				current_node->next_hash1 = ((next_dict_entry) & HASH1_MASK) >> 8;
				current_node->next_hash2 = (next_dict_entry) & HASH2_MASK;
				#ifndef RESETABLE_DICTIONARY
				}
				#endif
				time_to_add = 1;
			}
			else {
				// We did find the entry in the list, so now we have to go back to the
				// start to see if there is a longer string that works too
				time_to_add = 0;
				current_dict_entry = temp_dict_entry;
			}
		}
			
		if (time_to_add == 1) {
			time_to_add = 0;
				
			// Add to dictionary
			#ifndef RESETABLE_DICTIONARY
			if (next_dict_entry < MAX_DICT_ENTRIES) {
			#endif
			node[next_dict_entry].longer_string = 0;
			node[next_dict_entry].next_hash1 = 0;
			node[next_dict_entry].next_hash2 = 0;
			node[next_dict_entry++].entry = new_char;
			add_to_mc = 1;
			#ifndef RESETABLE_DICTIONARY
			}
			#endif
			
			#ifdef USE_MINICACHE
				if (mini_cache_structure[current_dict_entry & mini_dict_hash] == current_dict_entry) {
					// SUBLIST HIT
					// Part of one byte left or a while byte plus another part of one byte left?
					if (hit_bits > (8 - alignment)) {
						temp_entry = (((current_dict_entry & mini_dict_hash) << 1) | 0x01) << alignment;
						
						this->lzw_output_file_buffer[lzw_output_file_counter++] += temp_entry & 0xFF;
					
						temp_entry = temp_entry >> 8;
						this->lzw_output_file_buffer[lzw_output_file_counter] = temp_entry & 0xFF;
					}
					else {
						temp_entry = (((current_dict_entry & mini_dict_hash) << 1) | 0x01) << alignment;
						this->lzw_output_file_buffer[lzw_output_file_counter] += temp_entry & 0xFF;
					}
					
					alignment += hit_bits;
				}
				else {
					// SUBLIST MISS
					// Add to output buffer
					temp_entry = current_dict_entry << (alignment+1);
					
					this->lzw_output_file_buffer[lzw_output_file_counter++] += temp_entry & 0xFF;
					
					// Part of one byte left or a while byte plus another part of one byte left?
					if (size_of_entry + 1 - (8 - alignment) > 8) {
						temp_entry = current_dict_entry >> (7-alignment);	  // 7=>8-1 from miss tag
						this->lzw_output_file_buffer[lzw_output_file_counter++] = temp_entry & 0xFF;
						
						temp_entry = temp_entry >> 8;
					}
					else {
						temp_entry = current_dict_entry >> (7-alignment);
					}
					
					this->lzw_output_file_buffer[lzw_output_file_counter] = temp_entry & 0xFF;
					mini_cache_structure[current_dict_entry & mini_dict_hash] = current_dict_entry;
					alignment += size_of_entry+1;
				}
						
				// Add the newest dictionary entry to the list.
				#ifdef ADD_NEWEST
					#ifndef RESETABLE_DICTIONARY
						if (add_to_mc) {
					#endif
							mini_cache_structure[(next_dict_entry - 1) & mini_dict_hash] = next_dict_entry - 1;
					#ifndef RESETABLE_DICTIONARY
							add_to_mc = 0;
						}
					#endif
				#endif
			      
			#else	// If the program is not using the minicache.
				// Add to output buffer.
				temp_entry = current_dict_entry << alignment;
				this->lzw_output_file_buffer[lzw_output_file_counter++] += temp_entry & 0xFF;
					
				// Part of one byte left or a while byte plus another part of one byte left?
				if (size_of_entry - (8 - alignment) > 8) {
					temp_entry = current_dict_entry >> (8-alignment);
					this->lzw_output_file_buffer[lzw_output_file_counter++] = temp_entry & 0xFF;
					
					temp_entry = temp_entry >> 8;
				}
				else {
					temp_entry = current_dict_entry >> (8-alignment);
				}
					
				this->lzw_output_file_buffer[lzw_output_file_counter] = temp_entry & 0xFF;				
				alignment += size_of_entry;
			#endif
			
			while (alignment >= 8) {
				alignment -= 8;
			}
			
			if (alignment == 0) {
				lzw_output_file_counter++;
			}
							
			// Reset variables
			current_node = &(node[new_char]);
			current_dict_entry = new_char;
						
			if (((next_dict_entry - 1) >> size_of_entry) != 0) {
				size_of_entry++;
				
				#ifdef RESETABLE_DICTIONARY
					if (size_of_entry > MAX_ENTRY_BITS)
					{
						size_of_entry = 9;
						init_dictionary();
						next_dict_entry = 256;
					}
				#endif
			}
		}
		
		last_char = new_char;
	}
	
	// Add last string to output buffer.  This is pretty much identical to the block of code above,
	// so I really should make this into a function.
	#ifdef USE_MINICACHE
		if (mini_cache_structure[current_dict_entry & mini_dict_hash] == current_dict_entry) {
			// SUBLIST HIT
			// Part of one byte left or a while byte plus another part of one byte left?
			if (hit_bits > (8 - alignment)) {
				temp_entry = (((current_dict_entry & mini_dict_hash) << 1) | 0x01) << alignment;
				this->lzw_output_file_buffer[lzw_output_file_counter++] += temp_entry & 0xFF;
				
				temp_entry = temp_entry >> 8;
				this->lzw_output_file_buffer[lzw_output_file_counter] = temp_entry & 0xFF;
			}
			else {
				temp_entry = (((current_dict_entry & mini_dict_hash) << 1) | 0x01) << alignment;
				this->lzw_output_file_buffer[lzw_output_file_counter] += temp_entry & 0xFF;
			}
		}
		else {
			// SUBLIST MISS
			// Add to output buffer
			temp_entry = current_dict_entry << (alignment+1);
			this->lzw_output_file_buffer[lzw_output_file_counter++] += temp_entry & 0xFF;
				
			// Part of one byte left or a while byte plus another part of one byte left?
			if (size_of_entry + 1 - (8 - alignment) > 8) {
				temp_entry = current_dict_entry >> (7-alignment);	  // 7=>8-1 from miss tag
				this->lzw_output_file_buffer[lzw_output_file_counter++] = temp_entry & 0xFF;
				
				temp_entry = temp_entry >> 8;
			}
			else {
				temp_entry = current_dict_entry >> (7-alignment);
			}
					
			this->lzw_output_file_buffer[lzw_output_file_counter] = temp_entry & 0xFF;
		}
	#else
		temp_entry = current_dict_entry << alignment;
		this->lzw_output_file_buffer[lzw_output_file_counter++] += temp_entry & 0xFF;
				
		// Part of one byte left or a while byte plus another part of one byte left?
		if (size_of_entry - (8 - alignment) > 8) {
			temp_entry = current_dict_entry >> (8-alignment);
			this->lzw_output_file_buffer[lzw_output_file_counter++] = temp_entry & 0xFF;
				
			temp_entry = temp_entry >> 8;
		}
		else {
			temp_entry = current_dict_entry >> (8-alignment);
		}
				
		this->lzw_output_file_buffer[lzw_output_file_counter] = temp_entry & 0xFF;
	#endif
	
	// In case this actually expands our output.
	if (lzw_output_file_counter + 1 > bytes_in_data_in) {
		this->lzw_output_file_buffer[0] = 0;
		#ifndef STRUCTURED_TRANSFORM
			memcpy(&this->lzw_output_file_buffer[1], this->write_buffer, bytes_in_data_in);
		#else
			// Undo the structured transform if necessary.
			lzw_output_file_counter = 1;
			for (i=0; i<NUMBER_OF_READINGS; i++) {
				for (j=i; j < bytes_in_data_in; j+=NUMBER_OF_READINGS) {
					this->lzw_output_file_buffer[lzw_output_file_counter++] = this->write_buffer[j];	
					_stream_com_data[j] = this->write_buffer[j];
				}
			}
		#endif
		return bytes_in_data_in +1;
	}

	return (++lzw_output_file_counter);
}

//-------------------------------------------------------------------------------------
//función de descompresión S-LZW MC.
int compressor_slzw_mc::decompressorSLZW_MC(int _samples_amount, int *_data_out)
{
	unsigned int i, size_of_entry = 9, next_dict_entry, entry, last_entry=-1, hit_or_miss = MISS, right_shift = 0, num_bytes = 0;
	unsigned char alignment = 0, get_size = size_of_entry + 1, temp_letter;
	int bytes_in_data_in = sizeof(int)*_samples_amount;
	char write_buffer_aux[4];
	Undict_node *current_node, *unnode;

	// Decompression buffer.
	unsigned char array[200];
	unsigned char array_counter=0;	// I've decided that this is the max string length
	unsigned char alignment_bitmasks[] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

	unsigned char mini_dict_hash, mini_cache_size, hit_bits;
	
	#ifdef USE_MINICACHE
		// Holds the actual mini cache.
		unsigned int mini_cache_structure[MAX_SIZE_OF_MINI_CACHE];
	#endif
	
	#ifdef STRUCTURED_TRANSFORM
		unsigned int j, lzw_output_file_counter;
	#endif
	
	unnode = (Undict_node *) this->compression_chars;
    
	initDecompDictionary();
	next_dict_entry = 256;
	
	for (i = 0; i < MAX_SIZE_OF_MINI_CACHE; i++) {
		mini_cache_structure[i] = i;
	}
	
	mini_cache_size = this->lzw_output_file_buffer[0];
	hit_bits = 0;
	
	// Check to see if the compressor decided it was best to use the data in an uncompressed form.
	if (mini_cache_size == 0) {	
		
		for (int i1 = 0; i1 < (bytes_in_data_in - 1); i1++)
		{
			this->write_buffer[i1] = this->lzw_output_file_buffer[i1 + 1];
		}

		return bytes_in_data_in -1;
	}
	
	for (i = mini_cache_size; i != 0; i=i>>1) {
		hit_bits++;
	}
	
	mini_dict_hash = mini_cache_size - 1;
	
	if ((this->lzw_output_file_buffer[1] & 0x01) == HIT) {	// Whats up first?
		get_size = hit_bits;
		hit_or_miss = HIT;
	}
	
	entry = 0;
	alignment = 0;			// Looked at first bit already
	
	for (i=1; i<bytes_in_data_in; i++) {
		// Get entry
		if (get_size > 8) {
			entry += (this->lzw_output_file_buffer[i] << alignment);
			get_size -= 8;
			alignment += 8;
		} else {
			if (get_size > 0) {
				entry += (this->lzw_output_file_buffer[i] & alignment_bitmasks[get_size]) << alignment; 
			}
			
			entry = entry >> 1;					// Ditch hit or miss bit
			
			if (right_shift > 0) {
				entry = entry >> right_shift;
				entry = entry >> alignment;
				right_shift = 0;
			}
			
			if (hit_or_miss == MISS) {
				if (entry < next_dict_entry) {	// its in our dictionary
					current_node = &(unnode[entry]);
					temp_letter = current_node->entry;
					while (current_node->up != -1) {
						array[array_counter++] = current_node->entry;
						current_node = &(unnode[current_node->up]);
					}
					
					this->write_buffer[num_bytes++] = current_node->entry;
				
					while (array_counter > 0) {
						this->write_buffer[num_bytes++] = array[--array_counter];
					}
				} else { // its not in our dictionary yet
					current_node = &(unnode[last_entry]);
					temp_letter = current_node->entry;
					while (current_node->up != -1) {
						array[array_counter++] = current_node->entry;
						current_node = &(unnode[current_node->up]);
					}
					
					this->write_buffer[num_bytes++] = current_node->entry;
				
					while (array_counter > 0) {
						this->write_buffer[num_bytes++] = array[--array_counter];
					}
	
					// Reprint first character in the string as is the protocol for this case
					this->write_buffer[num_bytes++] = current_node->entry;
				}
				
				array_counter = 0;
				
				// Update subdictionary
				mini_cache_structure[entry & mini_dict_hash] = entry;
			}
			else {			// HIT
				entry = mini_cache_structure[entry];
				if (entry < next_dict_entry) {	// its in our dictionary
					current_node = &(unnode[entry]);
					temp_letter = current_node->entry;
					while (current_node->up != -1) {
						array[array_counter++] = current_node->entry;
						current_node = &(unnode[current_node->up]);
					}
					
					this->write_buffer[num_bytes++] = current_node->entry;
				
					while (array_counter > 0) {
						this->write_buffer[num_bytes++] = array[--array_counter];
					}
				}
				else { // its not in our dictionary yet
					current_node = &(unnode[last_entry]);
					temp_letter = current_node->entry;
					while (current_node->up != -1) {
						array[array_counter++] = current_node->entry;
						current_node = &(unnode[current_node->up]);
					}
					
					this->write_buffer[num_bytes++] = current_node->entry;
				
					while (array_counter > 0) {
						this->write_buffer[num_bytes++] = array[--array_counter];
					}
	
					// Reprint first character in the string as is the protocol for this case
					this->write_buffer[num_bytes++] = current_node->entry;
				}
				
				array_counter = 0;
			}
			
			// Create dictionary entry
			if (last_entry != -1) {
				if (last_entry > next_dict_entry) {
					return 0;
				}
				
				#ifndef RESETABLE_DICTIONARY
				if (next_dict_entry < MAX_DICT_ENTRIES) {
				#endif
					unnode[next_dict_entry].entry = current_node->entry;
					unnode[next_dict_entry++].up = last_entry;
					#ifdef ADD_NEWEST
						if (next_dict_entry < MAX_DICT_ENTRIES) {
							mini_cache_structure[(next_dict_entry) & mini_dict_hash] = next_dict_entry;			
						}
					#endif
				#ifndef RESETABLE_DICTIONARY
				}
				#endif
			
				if ((next_dict_entry >> size_of_entry) != 0) {
					#ifndef RESETABLE_DICTIONARY
					if (size_of_entry < MAX_ENTRY_BITS) {
					#endif
					size_of_entry++;
					#ifndef RESETABLE_DICTIONARY
					}
					#endif
					
					#ifdef RESETABLE_DICTIONARY
					if (size_of_entry > MAX_ENTRY_BITS) {
						size_of_entry = 9;
						init_decomp_dictionary();
						next_dict_entry = 256;
						entry = -1;			// Get ready to build a new dictionary
					}
					#endif
				}
			}
			else {
				#ifndef RESETABLE_DICTIONARY
				if (next_dict_entry < MAX_DICT_ENTRIES) {
				#endif
					#ifdef ADD_NEWEST
						mini_cache_structure[(next_dict_entry) & mini_dict_hash] = next_dict_entry;					
					#endif
				#ifndef RESETABLE_DICTIONARY
				}
				#endif
			}
			
			// Clean up
			last_entry = entry;
			if (get_size < 8) {
				entry = this->lzw_output_file_buffer[i] >> get_size;
				if ((entry & 0x01) == HIT) {
					if ((8-get_size) >= hit_bits) {		// all of the next entry is already here
						i--;		// So when we get the next byte, we get this one again
						entry = 0;
						alignment = get_size;
						right_shift = get_size;
						get_size += hit_bits;
					}	
					else {
						alignment = 8-get_size;
						get_size = hit_bits - alignment;
					}
					
					hit_or_miss = HIT;
				}
				else {
					alignment = 8-get_size;
					get_size = size_of_entry+1-alignment;
					hit_or_miss = MISS;
				}
			}
			else {
				if (i+1 < bytes_in_data_in) {
					if ((this->lzw_output_file_buffer[i+1] & 0x01) == HIT) {		// Whats up first?
						get_size = hit_bits;
						hit_or_miss = HIT;
					}
					else {
						get_size = size_of_entry+1;
						hit_or_miss = MISS;
					}
	
					entry = 0;
					alignment = 0;			// Looked at first bit already
				}				
				else {
					//fprintf(stderr, "Looking at EOF\n");
				}
			}
		}
	}
	
	#ifdef STRUCTURED_TRANSFORM
		// There is probably a smarter way to do this, but this was fast enough for the block size.
		lzw_output_file_counter = 0;
		for (i=0; i<NUMBER_OF_READINGS; i++) {
			for (j=i; j < num_bytes; j+=NUMBER_OF_READINGS) {
				this->lzw_output_file_buffer[lzw_output_file_counter++] = this->write_buffer[j];	
			}
		}
		
		for (int i1 = 0; i1 < num_bytes; i1++)
		{
			this->write_buffer[i1] = this->lzw_output_file_buffer[i1];
		}
	#endif

		//bucle para obtener datos de salida.
		for (int i1 = 1; i1 < _samples_amount; i1++)
		{
			//seteo de write_buffer_aux.
			write_buffer_aux[0] = this->write_buffer[i1 * 4];
			write_buffer_aux[1] = this->write_buffer[i1 * 4 + 1];
			write_buffer_aux[2] = this->write_buffer[i1 * 4 + 2];
			write_buffer_aux[3] = this->write_buffer[i1 * 4 + 3];

			//obtención de "data_out[i1]".
			_data_out[i1] = *(int *)& write_buffer_aux;
		}

		//limpieza de buffers.
		this->cleanBuffersSLZW();
	
	return num_bytes;
}

//-------------------------------------------------------------------------------------
//función para inicializar diccionario.
void compressor_slzw_mc::initDictionary()
{
	unsigned int j;
	Dict_node* node = (Dict_node *)this->compression_chars;

	for (j = 0; j<256; j++) {
		node[j].longer_string = 0;
		node[j].next_hash1 = ((j + 1) & HASH1_MASK) >> 8;
		node[j].next_hash2 = (j + 1) & HASH2_MASK;
		node[j].entry = j;
	}
}

//-------------------------------------------------------------------------------------
//función para inicializar diccionario para descompresión.
void compressor_slzw_mc::initDecompDictionary()
{
	unsigned int i;
	Undict_node* unnode = (Undict_node *) this->compression_chars;
	
	for (i=0; i<256; i++) {
		unnode[i].up = -1;
		unnode[i].entry = i;
	}
}

//-------------------------------------------------------------------------------------
//función para limpiar buffer de datos comprimidos.
int compressor_slzw_mc::cleanBuffersSLZW()
{
	//bucle para limpiar buffer "this->lzw_output_file_buffer".
	for (int i1 = 0; i1 < LZW_BUFFER_SIZE; i1++)
	{
		//limpieza.
		this->lzw_output_file_buffer[i1] = 0;
	}

	//bucle para limpiar buffer "this->write_buffer".
	for (int i1 = 0; i1 < BLOCK_SIZE; i1++)
	{
		//limpieza.
		this->write_buffer[i1] = 0;
	}
}
