/// JS equivalents of types used in the WASM module
let Gains = Float64Array;
let Samples = Float32Array;
let size_t = Uint32Array;

/// library wrapper, create with from_buffer or from_stream with a buffer or stream of libear.wasm
export class EARLibrary {
  constructor(module) {
    this.module = module;
    this.exp = module.instance.exports;

    this.text_encoder = new TextEncoder();

    this.exp.ear_init();
  }

  static async from_buffer(wasm_buffer) {
    const import_object = EARLibrary.#get_imports();

    let module = await WebAssembly.instantiate(wasm_buffer, import_object);

    return new EARLibrary(module);
  }

  static async from_stream(wasm_stream) {
    const import_object = EARLibrary.#get_imports();

    let module = await WebAssembly.instantiateStreaming(
      wasm_stream,
      import_object,
    );

    return new EARLibrary(module);
  }

  /// get a Layout object for a BS.2051 layout with the given name
  get_layout(name) {
    return this.#with_str(name, (name_ptr) => {
      let layout_ptr = this.exp.ear_layout_get(name_ptr);
      if (layout_ptr == 0) throw RangeError("unknown layout: " + name);
      return new Layout(this, layout_ptr);
    });
  }

  /// get a decorrelation filter for a given channel index in a Layout
  design_decorrelator(layout, channel_idx) {
    let length_ptr = this.exp.malloc(size_t.BYTES_PER_ELEMENT);
    let dec_ptr = this.exp.ear_design_decorrelator(
      layout.ptr,
      channel_idx,
      length_ptr,
    );

    let length_arr = new size_t(this.exp.memory.buffer, length_ptr, 1);
    let length = length_arr[0];

    let dec_arr = new Samples(this.exp.memory.buffer, dec_ptr, length);
    let dec_arr_copy = dec_arr.slice();

    this.exp.free(dec_ptr);
    this.exp.free(length_ptr);

    return dec_arr_copy;
  }

  /// get the delay in samples to compensate for the decorrelation filters
  decorrelator_compensation_delay() {
    return this.exp.ear_decorrelator_compensation_delay();
  }

  static #get_imports() {
    return {
      env: {
        ear_handle_exception(arg) {
          throw arg;
        },
        ear_handle_warning(arg) {
          console.log(arg);
        },
      },
    };
  }

  /// call cb with a pointer to the string s in memory, encoded as a c string,
  /// and free it when cb returns
  #with_str(s, cb) {
    let encoded = this.text_encoder.encode(s);
    let ptr = this.exp.malloc(encoded.length + 1);
    let arr = new Uint8Array(this.exp.memory.buffer, ptr);
    arr.set(encoded);
    arr[encoded.length] = 0;

    try {
      return cb(ptr);
    } finally {
      this.exp.free(ptr);
    }
  }
}

export class Layout {
  /// construct with a reference to an EARLibrary and a pointer -- use EARLibrary.get_layout
  constructor(lib, ptr) {
    this.lib = lib;
    this.ptr = ptr;
  }

  num_channels() {
    return this.lib.exp.ear_layout_num_channels(this.ptr);
  }
}

export class ObjectsTypeMetadata {
  /// construct with a reference to an EARLibrary
  constructor(lib) {
    this.lib = lib;
    this.ptr = this.lib.exp.ear_objects_type_metadata_new();
  }

  set_polar_position(azimuth, elevation, distance) {
    this.lib.exp.ear_objects_type_metadata_set_polar_position(
      this.ptr,
      azimuth,
      elevation,
      distance,
    );
  }

  set_gain(gain) {
    this.lib.exp.ear_objects_type_metadata_set_gain(this.ptr, gain);
  }

  set_extent(width, height, depth) {
    this.lib.exp.ear_objects_type_metadata_set_extent(
      this.ptr,
      width,
      height,
      depth,
    );
  }

  set_diffuse(diffuse) {
    this.lib.exp.ear_objects_type_metadata_set_diffuse(this.ptr, diffuse);
  }
}

export class GainCalculatorObjects {
  /// construct with a reference to an EARLibrary, and a Layout (returned from
  /// get_layout)
  constructor(lib, layout) {
    this.lib = lib;

    this.ptr = this.lib.exp.ear_gain_calculator_objects_new(layout.ptr);

    this.num_channels = layout.num_channels();
    this.direct = this.lib.exp.malloc(
      this.num_channels * Gains.BYTES_PER_ELEMENT,
    );
    this.diffuse = this.lib.exp.malloc(
      this.num_channels * Gains.BYTES_PER_ELEMENT,
    );
  }

  /// calculate gains for a given ObjectsTypeMetadata; returns an object with
  //"direct" and "diffuse" Gains
  calculate(otm) {
    this.lib.exp.ear_gain_calculator_objects_calc_gains(
      this.ptr,
      otm.ptr,
      this.num_channels,
      this.direct,
      this.diffuse,
    );

    let direct_arr = new Gains(
      this.lib.exp.memory.buffer,
      this.direct,
      this.num_channels,
    );
    let diffuse_arr = new Gains(
      this.lib.exp.memory.buffer,
      this.diffuse,
      this.num_channels,
    );

    return { direct: direct_arr.slice(), diffuse: diffuse_arr.slice() };
  }
}
