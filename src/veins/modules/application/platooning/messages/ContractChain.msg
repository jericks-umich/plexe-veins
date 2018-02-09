
cplusplus {{
#include "commpact_types.h"
}}

struct contract_chain_t;

// ContractChain will be composed of a contract_chain_t class and N signatures
// over the contract_chain_t object

packet ContractChain {
	unsigned char recipient;
	contract_chain_t contract_chain;
	//cp_ec256_signature_t sig_0; // added dynamically
	//cp_ec256_signature_t sig_4; // added dynamically
	//cp_ec256_signature_t sig_1; // added dynamically

	// signatures will be named "sig_" followed by the vehicle's position and may
	// be out of order (but will probably be in contract_chain.chain_order order)
}

